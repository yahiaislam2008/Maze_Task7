cmake_minimum_required(VERSION 3.8)
project(robot_controller)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveX.action"
  "action/Yaw.action"
)

install(
  DIRECTORY robot_controller/
  DESTINATION lib/python3.12/site-packages/robot_controller
)

install(
  PROGRAMS robot_controller/movement_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME movement_node
)

ament_package()
