cmake_minimum_required(VERSION 3.10)
project(map_memory)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED)   # ROS2 build tool
find_package(rclcpp REQUIRED)        # ROS2 C++ package
find_package(nav_msgs REQUIRED)      # OccupancyGrid and Odometry messages
find_package(geometry_msgs REQUIRED) # Quaternion message

# Compiles source files into a library
add_library(map_memory_lib
  src/map_memory_core.cpp
)

# Indicate to compiler where to search for header files
target_include_directories(map_memory_lib
  PUBLIC include
)

# Add ROS2 dependencies required by library
ament_target_dependencies(map_memory_lib
  rclcpp
)

# Create ROS2 node executable from source files
add_executable(map_memory_node
  src/map_memory_node.cpp
)

# Add ROS2 dependencies required by the node
ament_target_dependencies(map_memory_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# Link to the previously built library
target_link_libraries(map_memory_node
  map_memory_lib
)

# Copy executable to installation location
install(TARGETS
  map_memory_node
  DESTINATION lib/${PROJECT_NAME}
)

# Copy launch and config files to installation location
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()