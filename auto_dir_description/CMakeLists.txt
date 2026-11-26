cmake_minimum_required(VERSION 3.8)
project(auto_dir_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes rviz sdf urdf worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
