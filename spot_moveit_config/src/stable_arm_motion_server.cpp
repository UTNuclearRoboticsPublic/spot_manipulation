#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/utils/moveit_error_code.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <std_msgs/msg/string.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <spot_msgs/action/arm_cartesian_command.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>

class StableArmMotionServer : public rclcpp::Node {
public:
    StableArmMotionServer(std::string name, rclcpp::NodeOptions opts = rclcpp::NodeOptions{}):
    Node(name, opts),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
    {
        moveit_plan_client_ = create_client<moveit_msgs::srv::GetMotionPlan>("/spot_moveit/plan_kinematic_path");
        moveit_cartesian_client_ = create_client<moveit_msgs::srv::GetCartesianPath>("/spot_moveit/compute_cartesian_path");
        spot_driver_motion_client_ = rclcpp_action::create_client<spot_msgs::action::ArmCartesianCommand>(this, "/spot_manipulation_driver/arm_cartesian_command");

        using namespace std::placeholders;
        move_group_server_ = rclcpp_action::create_server<moveit_msgs::action::MoveGroup>(this, 
            "/spot_moveit/stable_move_action", 
            std::bind(&StableArmMotionServer::handleMoveActionRequest, this, _1, _2),
            std::bind(&StableArmMotionServer::handleMoveActionCancel, this, _1),
            std::bind(&StableArmMotionServer::handleMoveActionAccepted, this, _1)
        );

        trajectory_server_ = rclcpp_action::create_server<moveit_msgs::action::ExecuteTrajectory>(this,
            "/spot_moveit/execute_stable_trajectory",
            std::bind(&StableArmMotionServer::handleTrajectoryActionRequest, this, _1, _2),
            std::bind(&StableArmMotionServer::handleTrajectoryActionCancel, this, _1),
            std::bind(&StableArmMotionServer::handleTrajectoryActionAccepted, this, _1)
        );

        urdf_sub_ = create_subscription<std_msgs::msg::String>("/spot_driver/robot_description", rclcpp::QoS(1).transient_local(),
            std::bind(&StableArmMotionServer::receiveURDF, this, _1));

        srdf_sub_ = create_subscription<std_msgs::msg::String>("/spot_moveit/robot_description_semantic", rclcpp::QoS(1).transient_local(),
            std::bind(&StableArmMotionServer::receiveSRDF, this, _1));

        update_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this](){update();});
        init_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this](){init();});
    }

    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------

    void receiveURDF(std_msgs::msg::String::SharedPtr urdf_string) {
        urdf_string_ = urdf_string->data;
        urdf_sub_.reset();
    }

    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------

    void receiveSRDF(std_msgs::msg::String::SharedPtr srdf_string) {
        srdf_string_ = srdf_string->data;
        srdf_sub_.reset();
    }

    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------

    void init() {
        if (urdf_string_.empty() || srdf_string_.empty()) return;
        robot_model_loader::RobotModelLoader::Options opts;
        opts.urdf_string_ = urdf_string_;
        opts.srdf_string_ = srdf_string_;
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), opts);
        robot_model_ = robot_model_loader_->getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), robot_model_loader_);
        scene_monitor_->startSceneMonitor();
        scene_monitor_->startStateMonitor();
        init_timer_.reset();
    }

    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------

    rclcpp_action::GoalResponse handleMoveActionRequest(const rclcpp_action::GoalUUID&, moveit_msgs::action::MoveGroup::Goal::ConstSharedPtr goal) {
        RCLCPP_INFO(get_logger(), "Received execution request");
        if (isQueryActive()) {
            RCLCPP_WARN(get_logger(), "Cancelling previously active request");
            cancelActiveQuery();
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------

    rclcpp_action::CancelResponse handleMoveActionCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::MoveGroup>> goal) {
        auto result = std::make_shared<moveit_msgs::action::MoveGroup::Result>();
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
        goal->abort(result);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------

    void handleMoveActionAccepted (const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::MoveGroup>> goal) {
        // Make planning request of moveit
        if (!moveit_plan_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "Did not detect MoveGroup server wtihin 5 seconds");
            auto result = std::make_shared<moveit_msgs::action::MoveGroup::Result>();
            result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::COMMUNICATION_FAILURE;
            goal->abort(result);
        }

        auto planning_goal = std::make_shared<moveit_msgs::srv::GetMotionPlan::Request>();
        planning_goal->motion_plan_request = goal->get_goal()->request;
        moveit_plan_future_ = moveit_plan_client_->async_send_request(planning_goal);
        motion_plan_request_start_time_ = now();
    }

    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------

    rclcpp_action::GoalResponse handleTrajectoryActionRequest(const rclcpp_action::GoalUUID&, moveit_msgs::action::ExecuteTrajectory::Goal::ConstSharedPtr goal) {
        RCLCPP_INFO(get_logger(), "Received trajectory execution request");
        if (isQueryActive()) {
            RCLCPP_WARN(get_logger(), "Cancelling previously active request");
            cancelActiveQuery();
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------

    rclcpp_action::CancelResponse handleTrajectoryActionCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>> goal) {
        auto result = std::make_shared<moveit_msgs::action::ExecuteTrajectory::Result>();
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
        goal->abort(result);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------

    void handleTrajectoryActionAccepted (const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>> goal) {
        // Make planning request of moveit
        if (!spot_driver_motion_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "Did not detect Spot drvier action server wtihin 5 seconds");
            auto result = std::make_shared<moveit_msgs::action::ExecuteTrajectory::Result>();
            result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::COMMUNICATION_FAILURE;
            goal->abort(result);
        }

        spot_msgs::action::ArmCartesianCommand::Goal full_trajectory = generateFullTrajectory(goal->get_goal()->trajectory.joint_trajectory);
        spot_driver_motion_request_future_ = spot_driver_motion_client_->async_send_goal(full_trajectory);
        motion_request_start_time_ = now();
    }

    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------

    void cancelActiveQuery() {
        if (moveit_plan_future_.has_value()) {
            moveit_plan_client_->remove_pending_request(moveit_plan_future_.value());
            moveit_plan_future_.reset();
        }

        if (spot_driver_motion_request_future_.valid() || spot_driver_goal_handle_) {
            spot_driver_motion_client_->async_cancel_all_goals();
            spot_driver_motion_request_future_ = decltype(spot_driver_motion_request_future_){};
            spot_driver_goal_handle_.reset();
        }
    }

    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------

    void update() {
        // Check the status of an ongoing motion plan request
        if (moveit_plan_future_.has_value()) {
            std::future_status status = moveit_plan_future_->future.wait_for(std::chrono::seconds(0));
            if (status == std::future_status::ready) {
                moveit_msgs::srv::GetMotionPlan::Response::SharedPtr resp = moveit_plan_future_->get();
                moveit_plan_future_.reset();
                if (resp->motion_plan_response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                    RCLCPP_ERROR(get_logger(), "Failed to plan path: %s", moveit::core::error_code_to_string(resp->motion_plan_response.error_code).c_str());
                    return;
                } else {
                    RCLCPP_INFO(get_logger(), "Received a trajectory with %zd waypoints. Converting to stable trajectory", resp->motion_plan_response.trajectory.joint_trajectory.points.size());
                }

                // Create and send full trajectory
                try {
                    spot_msgs::action::ArmCartesianCommand::Goal full_trajectory = generateFullTrajectory(resp->motion_plan_response.trajectory.joint_trajectory);
                    spot_driver_motion_request_future_ = spot_driver_motion_client_->async_send_goal(full_trajectory);
                    motion_request_start_time_ = now();
                } catch (tf2::TransformException& e) {
                    RCLCPP_ERROR(get_logger(), "Unable to transform trajectory to the odom frame: %s", e.what());
                    cancelActiveQuery();
                }        
            } else if (status == std::future_status::timeout) {
                const float elapsed_time = (now() - motion_plan_request_start_time_).seconds();
                if (elapsed_time > 5.0) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5, "Still waiting on response from MoveIt planning server");
                }
            }
        }

        // If there is no motion plan request check to see if we have a motion action request
        else if (spot_driver_motion_request_future_.valid()) {
            std::future_status status = spot_driver_motion_request_future_.wait_for(std::chrono::seconds(0));
            if (status == std::future_status::ready) {
                RCLCPP_INFO(get_logger(), "Starting robot motion");
                spot_driver_goal_handle_ = spot_driver_motion_request_future_.get();
                spot_driver_motion_request_future_ = decltype(spot_driver_motion_request_future_){};
                motion_start_time_ = now();
            } else if (status == std::future_status::timeout) {
                const float elapsed_time = (now() - motion_request_start_time_).seconds();
                if (elapsed_time > 5.0) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5, "Still waiting on response from Spot driver");
                }
            }
        }

        // If there is no motion action request, check to see if there is an active motion goal
        else if (spot_driver_goal_handle_) {
            // Handle feedback or status
            int8_t goal_status = spot_driver_goal_handle_->get_status();
            switch (goal_status){
                case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
                case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
                case action_msgs::msg::GoalStatus::STATUS_CANCELING:
                {
                    // Check to see if the motion is still valid
                    planning_scene_monitor::LockedPlanningSceneRO scene_reader(scene_monitor_);
                    if (!scene_reader->isPathValid(active_trajectory_start_state_, active_trajectory_)) {
                        RCLCPP_WARN(get_logger(), "The active trajectory is no longer collision free. Aborting");
                        cancelActiveQuery();
                    }

                    const rclcpp::Duration elapsed_time = now() - motion_start_time_;
                    if (elapsed_time.seconds() > 10.0) {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5, "Still waiting for Spot driver to complete motion");
                    }
                    return;
                }
                
                case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
                    RCLCPP_WARN(get_logger(), "Stable motion returned status UNKNOWN, aborting");
                    cancelActiveQuery();
                    return;
                
                case action_msgs::msg::GoalStatus::STATUS_ABORTED:
                case action_msgs::msg::GoalStatus::STATUS_CANCELED:
                    RCLCPP_WARN(get_logger(), "Stable motion failed");
                    spot_driver_goal_handle_.reset();
                    return;

                case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
                    RCLCPP_INFO(get_logger(), "Stable arm motion complete");
                    spot_driver_goal_handle_.reset();
                    return;
            }
        }
    }

    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------

    spot_msgs::action::ArmCartesianCommand::Goal generateFullTrajectory(const trajectory_msgs::msg::JointTrajectory& joint_traj) {
        // Mark this as the active trajectory
        active_trajectory_.joint_trajectory = joint_traj;
        planning_scene_monitor::LockedPlanningSceneRO scene_reader(scene_monitor_);
        moveit::core::robotStateToRobotStateMsg(scene_reader->getCurrentState(), active_trajectory_start_state_);

        spot_msgs::action::ArmCartesianCommand::Goal spot_arm_command;
        spot_arm_command.joint_waypoints = joint_traj;
        spot_arm_command.x_axis_mode = spot_arm_command.AXIS_MODE_POSITION;
        spot_arm_command.y_axis_mode = spot_arm_command.AXIS_MODE_POSITION;
        spot_arm_command.z_axis_mode = spot_arm_command.AXIS_MODE_POSITION;
        spot_arm_command.header.frame_id = "odom";

        // Transform the targets into the odom frame for spot
        geometry_msgs::msg::TransformStamped odom_tform_body_ros = tf_buffer_.lookupTransform(
            "odom", robot_model_->getRootLinkName(), rclcpp::Time(0)
        );
        const Eigen::Isometry3d odom_tform_body = tf2::transformToEigen(odom_tform_body_ros);

        for (const trajectory_msgs::msg::JointTrajectoryPoint& joint_pos : joint_traj.points) {
            // Update the arm state
            moveit::core::JointModelGroup* arm_group = robot_model_->getJointModelGroup("arm");
            robot_state_->setJointGroupPositions(arm_group, joint_pos.positions);
            robot_state_->setJointGroupVelocities(arm_group, joint_pos.velocities);
            robot_state_->setJointGroupAccelerations(arm_group, joint_pos.accelerations);
            robot_state_->updateLinkTransforms();

            // Get the end effector position and set it in the trajectory
            const Eigen::Isometry3d ee_pose_in_body = robot_state_->getGlobalLinkTransform("arm0_hand");

            // Convert to the odom frame
            const Eigen::Isometry3d ee_pose_in_odom = odom_tform_body * ee_pose_in_body;
            spot_arm_command.waypoints.push_back(tf2::toMsg(ee_pose_in_odom));
            spot_arm_command.timestamps.push_back(rclcpp::Duration(joint_pos.time_from_start).seconds());
        }

        return spot_arm_command;
    }

    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------

    bool isQueryActive() {
        return moveit_plan_future_.has_value() || spot_driver_motion_request_future_.valid() || spot_driver_goal_handle_;
    }

private:
    // Spin timer to check on active requests
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Timer and subscriptions to load the robot model
    rclcpp::TimerBase::SharedPtr init_timer_;
    std::string urdf_string_;
    std::string srdf_string_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr srdf_sub_;

    // Transforms
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Robot Model
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;

    // Collision Monitoring
    moveit_msgs::msg::RobotTrajectory active_trajectory_;
    moveit_msgs::msg::RobotState active_trajectory_start_state_;
    planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;

    // MoveIt Planning Requests
    rclcpp::Time motion_plan_request_start_time_;
    rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr moveit_plan_client_;
    rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr moveit_cartesian_client_;
    std::optional<rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::FutureAndRequestId> moveit_plan_future_;

    // Spot Motion Requests
    rclcpp::Time motion_request_start_time_;
    rclcpp_action::Client<spot_msgs::action::ArmCartesianCommand>::SharedPtr spot_driver_motion_client_;
    std::shared_future<rclcpp_action::ClientGoalHandle<spot_msgs::action::ArmCartesianCommand>::SharedPtr> spot_driver_motion_request_future_;
    rclcpp_action::ClientGoalHandle<spot_msgs::action::ArmCartesianCommand>::SharedPtr spot_driver_goal_handle_;
    rclcpp::Time motion_start_time_;

    // Advertised MoveIt Server
    rclcpp_action::Server<moveit_msgs::action::MoveGroup>::SharedPtr move_group_server_;
    rclcpp_action::Server<moveit_msgs::action::ExecuteTrajectory>::SharedPtr trajectory_server_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StableArmMotionServer>("stable_arm_motion_server");
    rclcpp::spin(node);
    return 0;
}