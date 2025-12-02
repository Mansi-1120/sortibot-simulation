cmake_minimum_required(VERSION 3.8)
project(sortibot_gazebo)

find_package(ament_cmake REQUIRED)

# Install launch files, worlds, and (optionally) models into share/sortibot_gazebo
install(
  DIRECTORY
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

