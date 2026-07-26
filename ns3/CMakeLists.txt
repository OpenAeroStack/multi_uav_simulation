# CMakeLists.txt for the NS-3 scenarios.
#
# Copy the whole ns3/ folder into an NS-3 scratch subdirectory:
#   cp -r ns3 <ns-3.38>/scratch/multi_uav_simulation
# then build from the NS-3 root:
#   ./ns3 build three_uav_tapbridge_integrated
#
# Source ROS 2 Humble before configuring NS-3 so CMake can find rclcpp and
# std_msgs.

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ACTIVE TARGET: integrated GCS + three-UAV obstacle-aware scenario.
build_exec(
  EXECNAME three_uav_tapbridge_integrated
  EXECUTABLE_DIRECTORY_PATH
    ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/multi_uav_simulation
  SOURCE_FILES
    three_uav_tapbridge_integrated.cc
    dynamic_obstacle_loss_model.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libwifi}
    ${libmobility}
    ${libtap-bridge}
    ${libpropagation}
    ${libstats}
    ${libnetanim}
    ${ns3-libs}
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
)

# LEGACY TARGET: predecessor obstacle-loss scenario. Retained for validation
# history only; new work should use three_uav_tapbridge_integrated.
build_exec(
  EXECNAME three_uav_tapbridge_obstacle_loss
  EXECUTABLE_DIRECTORY_PATH
    ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/multi_uav_simulation
  SOURCE_FILES
    three_uav_tapbridge_obstacle_loss.cc
    dynamic_obstacle_loss_model.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libwifi}
    ${libmobility}
    ${libtap-bridge}
    ${libpropagation}
    ${libstats}
    ${ns3-libs}
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
)

# LEGACY TARGET: original real-time scenario from the small-city branch.
# Retained for launcher compatibility; it is not the active obstacle-aware
# target.
build_exec(
  EXECNAME three-uav
  EXECUTABLE_DIRECTORY_PATH
    ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/multi_uav_simulation
  SOURCE_FILES
    three_uav_tapbridge_rt.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libcsma}
    ${libtap-bridge}
    ${libnetanim}
    ${libflow-monitor}
    ${libmobility}
    ${libwifi}
    ${libpropagation}
    ${ns3-libs}
)
