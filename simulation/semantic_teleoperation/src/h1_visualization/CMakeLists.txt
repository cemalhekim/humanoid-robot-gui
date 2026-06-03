cmake_minimum_required(VERSION 3.8)
project(h1_visualization)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/zero_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
