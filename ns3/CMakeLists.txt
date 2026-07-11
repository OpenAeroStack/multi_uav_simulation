# CMakeLists.txt for the NS-3 obstacle-loss scenario.
#
# Copy the whole ns3/ folder into your NS-3 tree as a scratch subdirectory:
#   cp -r ns3  <ns-3.38>/scratch/multi_uav_simulation
# then build from the NS-3 root:
#   ./ns3 build three_uav_tapbridge_obstacle_loss
#
# Requires a ROS 2 (Humble) environment to be sourced before ./ns3 configure,
# because the scenario links against rclcpp / std_msgs.

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

build_exec(
  EXECNAME three_uav_tapbridge_obstacle_loss
  EXECUTABLE_DIRECTORY_PATH ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/multi_uav_simulation
  SOURCE_FILES
    three_uav_tapbridge_obstacle_loss.cc
    dynamic_obstacle_loss_model.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libwifi}
    ${libmobility}
    ${libtap-bridge}
    ${libpropagation}
    ${libinternet}
    ${libstats}
    ${ns3-libs}
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
)
