# This CMakeLists.txt is intended to be used with the ns-3 (CMake) build.
#
# Recommended usage (ns-3.38+):
# 1) Create a directory in your ns-3 tree:  scratch/three-uav/
# 2) Copy this file into:                scratch/three-uav/CMakeLists.txt
# 3) Copy three_uav_tapbridge_rt.cc into: scratch/three-uav/three_uav_tapbridge_rt.cc
# 4) Build/run:
#      ./ns3 build three-uav
#      sudo ./ns3 run --enable-sudo "three-uav --tap0=tap-gcs --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --simDurationSec=60 --animFile=/tmp/three_uav_anim.xml"
# in scenario we using the nakamiPropogation model and the LongdistancePropogation model we hace to use libpropagation.
# here libcsma is not using.
 


build_exec(
  EXECNAME three-uav
  SOURCE_FILES three_uav_tapbridge_rt.cc
  LIBRARIES_TO_LINK
    # Required: core, network, wifi, mobility, tap-bridge, flow-monitor, internet.
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
    EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/three-uav
)
