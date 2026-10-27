#!/usr/bin/env bash
# Source in a real G1 navigation terminal. Domain 0 is the robot's DDS domain.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo 'Use: source scripts/activate_g1_real.sh' >&2
    exit 1
fi
_g1_real_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
if ! source "$_g1_real_root/scripts/activate_g1_nav.sh"; then
    return 1
fi
_g1_cyclone="$_g1_real_root/.cache-g1/ros-cyclone/root/opt/ros/humble"
if [[ ! -f "$_g1_cyclone/lib/librmw_cyclonedds_cpp.so" ]]; then
    echo 'Missing local Cyclone ROS middleware; run scripts/setup_g1_real.sh.' >&2
    return 1
fi
export AMENT_PREFIX_PATH="$_g1_cyclone:$AMENT_PREFIX_PATH"
export CMAKE_PREFIX_PATH="$_g1_cyclone:${CMAKE_PREFIX_PATH:-}"
export PATH="$_g1_cyclone/bin:$PATH"
export PYTHONPATH="$_g1_cyclone/local/lib/python3.10/dist-packages:$_g1_cyclone/lib/python3.10/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$_g1_cyclone/lib:$_g1_cyclone/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${G1_REAL_ROS_DOMAIN_ID:-0}"
export G1_ROBOT_ADDRESS="${G1_ROBOT_ADDRESS:-192.168.123.222}"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface address=\"$G1_ROBOT_ADDRESS\"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>"
if [[ -f "$_g1_real_root/.cache-g1/real-ros-build/install/local_setup.bash" ]]; then
    source "$_g1_real_root/.cache-g1/real-ros-build/install/local_setup.bash"
fi
echo "G1 real navigation: CycloneDDS / local $G1_ROBOT_ADDRESS / domain $ROS_DOMAIN_ID"
unset _g1_real_root _g1_cyclone
