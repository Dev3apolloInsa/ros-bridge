cmake_minimum_required(VERSION 3.5)
project(carla_trajectory_tracking)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(publish_waypoints src/waypoints_loader.cpp)
ament_target_dependencies(publish_waypoints rclcpp carla_msgs nav_msgs geometry_msgs)

add_executable(listener_base_waypoints src/subscrib_to_base_waypoints.cpp)
ament_target_dependencies(listener_base_waypoints rclcpp carla_msgs nav_msgs geometry_msgs)

add_executable(waypoints_updater src/waypoint_updater.cpp)
ament_target_dependencies(waypoints_updater rclcpp std_msgs carla_msgs geometry_msgs)

#add_executable(executor_example_3_node src/waypoint_updater.cpp)
#ament_target_dependencies(executor_example_3_node rclcpp nav_msgs std_msgs carla_msgs geometry_msgs)



install(TARGETS
  publish_waypoints
  listener_base_waypoints
  waypoints_updater
#  executor_example_3_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
