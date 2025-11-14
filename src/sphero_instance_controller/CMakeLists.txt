cmake_minimum_required(VERSION 3.8)
project(sphero_instance_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SpheroCommand.msg"
  "msg/SpheroHeading.msg"
  "msg/SpheroLED.msg"
  "msg/SpheroMatrix.msg"
  "msg/SpheroRoll.msg"
  "msg/SpheroSensor.msg"
  "msg/SpheroSpeed.msg"
  "msg/SpheroSpin.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Install Python modules
install(
  DIRECTORY ${PROJECT_NAME}/
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install Python executables
install(
  PROGRAMS
    ${PROJECT_NAME}/sphero_instance_device_controller_node.py
    ${PROJECT_NAME}/sphero_instance_task_controller_node.py
    ${PROJECT_NAME}/sphero_instance_statemachine_controller_node.py
    ${PROJECT_NAME}/sphero_instance_websocket_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install web interface templates and static files
install(
  DIRECTORY ${PROJECT_NAME}/templates
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}
)

install(
  DIRECTORY ${PROJECT_NAME}/static
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
