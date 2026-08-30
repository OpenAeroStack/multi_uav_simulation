cmake_minimum_required(VERSION 3.8)
project(multi_uav_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

add_library(obstacle_raycast_plugin SHARED
  src/obstacle_raycast_plugin.cc)

target_include_directories(obstacle_raycast_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${GAZEBO_INCLUDE_DIRS})

target_link_libraries(obstacle_raycast_plugin
  ${GAZEBO_LIBRARIES})

ament_target_dependencies(obstacle_raycast_plugin
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_dev
  gazebo_ros)

install(TARGETS obstacle_raycast_plugin
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_libraries(obstacle_raycast_plugin)
ament_package()
