# CMakeLists.txt for the NS-3 scenarios.
#
# Copy the whole ns3/ folder into your NS-3 tree as a scratch subdirectory:
#   cp -r ns3  <ns-3.38>/scratch/multi_uav_simulation
# then build from the NS-3 root:
#   ./ns3 build three_uav_tapbridge_integrated        <- the current scenario
#   ./ns3 build three_uav_tapbridge_obstacle_loss     <- 3-node predecessor
#
# Requires a ROS 2 (Humble) environment to be sourced before ./ns3 configure,
# because the scenarios link against rclcpp / std_msgs.

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Current scenario: 4 nodes (GCS + 3 UAVs), 802.11a, SNR sniffer, NetAnim.
# Extra lib vs. the 3-node target below: netanim. FlowMonitor is deliberately
# NOT linked -- it cannot see TapBridge UseLocal traffic (see the .cc).
build_exec(
  EXECNAME three_uav_tapbridge_integrated
  EXECUTABLE_DIRECTORY_PATH ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/multi_uav_simulation
  SOURCE_FILES
    three_uav_tapbridge_integrated.cc
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
    ${libnetanim}
    ${ns3-libs}
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
)

# Predecessor: 3 UAVs, no GCS node. Kept because the validation history and the
# older launch notes refer to it; new work should use the target above.
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
