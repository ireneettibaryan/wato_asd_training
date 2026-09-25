cmake_minimum_required(VERSION 3.10)
project(control)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Compile control core into a library
add_library(control_lib
  src/control_core.cpp
)

# Indicate where header files are located
target_include_directories(control_lib
  PUBLIC include
)

# Dependencies required by library
ament_target_dependencies(control_lib
  rclcpp
)

# Create ROS2 control executable
add_executable(control_node
  src/control_node.cpp
)

# Dependencies required by the control node
ament_target_dependencies(control_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# Link the control core library
target_link_libraries(control_node
  control_lib
)

# Install executable
install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()