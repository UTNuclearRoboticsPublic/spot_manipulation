cmake_minimum_required(VERSION 3.22)
project(spot_moveit_config)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(spot_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)

add_executable(stable_arm_motion_server src/stable_arm_motion_server.cpp)
ament_target_dependencies(stable_arm_motion_server moveit_msgs spot_msgs moveit_ros_planning)

install(
  TARGETS stable_arm_motion_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
