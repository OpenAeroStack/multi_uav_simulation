# multi_uav_gazebo_plugins/CMakeLists.txt


cmake_minimum_required(VERSION 3.8)
project(multi_uav_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)   # Gazebo Classic 11 ROS2 integration package
find_package(gazebo_ros REQUIRED)

add_library(gazebo_obstacle_plugin SHARED
  src/gazebo_obstacle_plugin.cc)

target_include_directories(gazebo_obstacle_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${GAZEBO_INCLUDE_DIRS})

target_link_libraries(gazebo_obstacle_plugin
  ${GAZEBO_LIBRARIES})

ament_target_dependencies(gazebo_obstacle_plugin
  rclcpp std_msgs geometry_msgs gazebo_dev gazebo_ros)

install(TARGETS gazebo_obstacle_plugin
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_libraries(gazebo_obstacle_plugin)
ament_package()