cmake_minimum_required(VERSION 3.10)
project(planner)

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

# Compile planner core into a library
add_library(planner_lib
  src/planner_core.cpp
)

# Indicate where header files are located
target_include_directories(planner_lib
  PUBLIC include
)

# Dependencies required by the library
ament_target_dependencies(planner_lib
  rclcpp
)

# Create the ROS2 planner executable
add_executable(planner_node
  src/planner_node.cpp
)

# ROS2 dependencies required by the planner node
ament_target_dependencies(planner_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# Link the planner core library
target_link_libraries(planner_node
  planner_lib
)

# Install the executable
install(TARGETS
  planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()