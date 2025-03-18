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
    auto marker_pub_raw = node->create_publisher<visualization_msgs::msg::MarkerArray>("/spot_curobo_config/raw_collision_spheres", latching_qos);
    auto marker_pub_self = node->create_publisher<visualization_msgs::msg::MarkerArray>("/spot_curobo_config/self_collision_spheres", latching_qos);
    auto marker_pub_inflated = node->create_publisher<visualization_msgs::msg::MarkerArray>("/spot_curobo_config/inflated_collision_spheres", latching_qos);

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    rclcpp::Rate loop_rate(10);

    const std::string file_path = std::filesystem::path(ament_index_cpp::get_package_share_directory("spot_curobo_config")).append("config").append("spot.xrdf").string();

    RCLCPP_INFO(node->get_logger(), "Loading config from file \"%s\"", file_path.c_str());
    YAML::Node robot_config = YAML::LoadFile(file_path);

    const std::string base_link = "base_footprint";

    visualization_msgs::msg::MarkerArray raw_array;
    visualization_msgs::msg::MarkerArray self_array;
    visualization_msgs::msg::MarkerArray inflated_array;
    visualization_msgs::msg::Marker marker;
    marker.action = marker.ADD;
    marker.type = marker.SPHERE;
    marker.color.r = 1.0;
    marker.color.a = 0.5;
    marker.header.frame_id = base_link;
    marker.header.stamp = rclcpp::Time(0); 

    while (rclcpp::ok()) {
        const rclcpp::Time now = node->now();
        auto collision_spheres = robot_config["geometry"]["spot_collision_spheres"]["spheres"];
        int idx = 0;
        for (const auto& item : collision_spheres) {
            const std::string child_link = item.first.as<std::string>();

            try{
                geometry_msgs::msg::TransformStamped base_link_tform_child_link = tf_buffer.lookupTransform(
                    base_link,
                    child_link,
                    now, 
                    std::chrono::milliseconds(500)
                );

                for (const auto& sphere : item.second) {
                    visualization_msgs::msg::Marker& new_marker = raw_array.markers.emplace_back() = marker;
                    geometry_msgs::msg::Pose& sphere_pose = new_marker.pose;
                    sphere_pose.position.x = sphere["center"][0].as<double>();
                    sphere_pose.position.y = sphere["center"][1].as<double>();
                    sphere_pose.position.z = sphere["center"][2].as<double>();
                    new_marker.scale.x =
                    new_marker.scale.y = 
                    new_marker.scale.z = 2*sphere["radius"].as<double>();
                    new_marker.id = idx++;

                    tf2::doTransform(sphere_pose, sphere_pose, base_link_tform_child_link);

                    YAML::Node self_buffer = robot_config["self_collision"]["buffer_distance"][child_link];
                    visualization_msgs::msg::Marker self_collision_marker = new_marker;
                    self_collision_marker.scale.x =
                    self_collision_marker.scale.y = 
                    self_collision_marker.scale.z = new_marker.scale.z + (self_buffer ? self_buffer.as<float>() : 0.0f);
                    self_array.markers.push_back(self_collision_marker);

                    YAML::Node inflation_buffer = robot_config["collision"]["buffer_distance"][child_link];
                    visualization_msgs::msg::Marker inflated_marker = new_marker;
                    inflated_marker.scale.x =
                    inflated_marker.scale.y = 
                    inflated_marker.scale.z = new_marker.scale.z + (inflation_buffer ? inflation_buffer.as<float>() : 0.0f);
                    inflated_array.markers.push_back(inflated_marker);
                }
            } catch (tf2::LookupException& e) {
                RCLCPP_WARN(node->get_logger(), "%s", e.what());
                continue;
            }
        }

        marker_pub_raw->publish(raw_array);
        marker_pub_self->publish(self_array);
        marker_pub_inflated->publish(inflated_array);

        raw_array.markers.clear();
        self_array.markers.clear();
        inflated_array.markers.clear();

        loop_rate.sleep();
    }
}