cmake_minimum_required(VERSION 3.8)
project(multirobot_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FleetRobot.msg"
  "msg/FleetState.msg"
  "msg/SimilarityCue.msg"
  "msg/ProximityCue.msg"
  "msg/FleetPolicy.msg"
  "srv/Register.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_package()
