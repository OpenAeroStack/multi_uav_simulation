# This CMakeLists.txt is intended to be used with the ns-3 (CMake) build.
#
# Recommended usage (ns-3.38+):
# 1) Create a directory in your ns-3 tree:  scratch/three-uav/
# 2) Copy this file into:                scratch/three-uav/CMakeLists.txt
# 3) Copy three_uav_tapbridge_rt.cc into: scratch/three-uav/three_uav_tapbridge_rt.cc
# 4) Build/run:
#      ./ns3 run "three-uav --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --simDurationSec=60 --animFile=three_uav_anim.xml"

build_exec(
  EXECNAME three-uav
  SOURCE_FILES three_uav_tapbridge_rt.cc
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
  EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/three-uav
)
