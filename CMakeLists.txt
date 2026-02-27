cmake_minimum_required(VERSION 3.8)
project(red_rover_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  worlds
  launch
  rviz
  config
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
