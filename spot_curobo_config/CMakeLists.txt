cmake_minimum_required(VERSION 3.22)
project(spot_curobo_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(
  DIRECTORY 
    meshes
    urdf
    usd
  DESTINATION share/${PROJECT_NAME}
)
