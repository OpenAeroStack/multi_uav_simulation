# ~/ns-3.38/scratch/multi_uav/CMakeLists.txt
#Add this file to the scratch folder along with other ns3 scripts

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

build_lib_example(
  NAME three_uav_tapbridge_rt
  SOURCE_FILES
    three_uav_tapbridge_rt.cc
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
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
)
