#include <thread>
#include <filesystem>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sphere_visualization");

    rclcpp::QoS latching_qos = rclcpp::QoS(1).transient_local();
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/spot_curobo_config/sphere_visualization", latching_qos);

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    rclcpp::Rate loop_rate(10);

    const std::string file_path = std::filesystem::path(ament_index_cpp::get_package_share_directory("spot_curobo_config")).append("spot_curobo.yaml").string();

    RCLCPP_INFO(node->get_logger(), "Loading config from file \"%s\"", file_path.c_str());
    YAML::Node robot_config = YAML::LoadFile(file_path);

    const std::string base_link = robot_config["robot_cfg"]["kinematics"]["base_link"].as<std::string>(); 

    visualization_msgs::msg::MarkerArray m_array;
    visualization_msgs::msg::Marker marker;
    marker.action = marker.ADD;
    marker.type = marker.SPHERE;
    marker.color.r = 1.0;
    marker.color.a = 0.5;
    marker.header.frame_id = base_link;
    marker.header.stamp = rclcpp::Time(0); 

    while (rclcpp::ok()) {
        const rclcpp::Time now = node->now();
        auto collision_spheres = robot_config["robot_cfg"]["kinematics"]["collision_spheres"];
        int idx = 0;
        for (const auto& item : collision_spheres) {
            const std::string child_link = item.first.as<std::string>();

            geometry_msgs::msg::TransformStamped base_link_tform_child_link = tf_buffer.lookupTransform(
                base_link,
                child_link,
                now, 
                std::chrono::milliseconds(500)
            );

            for (const auto& sphere : item.second) {
                visualization_msgs::msg::Marker& new_marker = m_array.markers.emplace_back() = marker;
                geometry_msgs::msg::Pose& sphere_pose = new_marker.pose;
                sphere_pose.position.x = sphere["center"][0].as<double>();
                sphere_pose.position.y = sphere["center"][1].as<double>();
                sphere_pose.position.z = sphere["center"][2].as<double>();
                new_marker.scale.x =
                new_marker.scale.y = 
                new_marker.scale.z = 2*sphere["radius"].as<double>();
                new_marker.id = idx++;

                tf2::doTransform(sphere_pose, sphere_pose, base_link_tform_child_link);
            }
        }

        marker_pub->publish(m_array);
        m_array.markers.clear();

        loop_rate.sleep();
    }
}