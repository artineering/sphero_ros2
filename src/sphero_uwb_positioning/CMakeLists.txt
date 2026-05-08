cmake_minimum_required(VERSION 3.8)
project(sphero_uwb_positioning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install Python modules
install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/ble_position_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ble_position_node
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
