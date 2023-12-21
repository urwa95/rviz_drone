cmake_minimum_required(VERSION 3.8)
project(my_drone_pkg)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Add executable for base station node
add_executable(base_station_node src/base_station_node.cpp)
ament_target_dependencies(base_station_node
  rclcpp
  geometry_msgs
  tf2_ros
)

# Add executable for drone node
add_executable(drone_node src/drone_node.cpp)
ament_target_dependencies(drone_node
  rclcpp
  geometry_msgs
  tf2_ros
)

# Install the executables
install(TARGETS
  base_station_node
  drone_node
  DESTINATION lib/${PROJECT_NAME}
)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Call ament_package()
ament_package()

