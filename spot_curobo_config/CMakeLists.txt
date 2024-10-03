cmake_minimum_required(VERSION 3.22)
project(spot_curobo_config)

find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(visualize_spheres src/visualize_spheres.cpp)
ament_target_dependencies(visualize_spheres tf2_ros visualization_msgs tf2_geometry_msgs)
target_link_libraries(visualize_spheres ${YAML_CPP_LIBRARIES})

install(
  DIRECTORY 
    meshes
    urdf
    usd
    config
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    spot_curobo.yaml
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  TARGETS
    visualize_spheres
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
