cmake_minimum_required(VERSION 3.8)
project(turtlebot_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
