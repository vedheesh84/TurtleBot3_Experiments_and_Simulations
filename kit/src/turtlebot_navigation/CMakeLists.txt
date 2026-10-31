cmake_minimum_required(VERSION 3.8)
project(turtlebot_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch behavior_trees rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
