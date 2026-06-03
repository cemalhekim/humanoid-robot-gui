cmake_minimum_required(VERSION 3.8)
project(h1_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/scripted_walk.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
