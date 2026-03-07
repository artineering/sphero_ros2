cmake_minimum_required(VERSION 3.8)
project(sphero_uwb_positioning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SpheroUWBStatus.msg"
  DEPENDENCIES std_msgs
)

# Install Python modules manually (rosidl_generate_interfaces already calls ament_python_install_package)
install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/sphero_uwb_positioning_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sphero_uwb_positioning_node
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
