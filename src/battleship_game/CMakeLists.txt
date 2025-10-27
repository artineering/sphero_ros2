cmake_minimum_required(VERSION 3.8)
project(battleship_game)

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

# Generate messages (only the working ones)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Attack.msg"
  "msg/AttackResult.msg"
  "msg/NewBoard.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Install Python modules
install(
  DIRECTORY ${PROJECT_NAME}/
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install Python scripts from battleship_game directory
install(
  PROGRAMS
    ${PROJECT_NAME}/game_controller_node.py
    ${PROJECT_NAME}/human_controller_node.py
    ${PROJECT_NAME}/sphero_agent_node.py
    ${PROJECT_NAME}/test_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
