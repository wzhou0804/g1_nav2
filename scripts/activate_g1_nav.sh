#!/usr/bin/env bash
# Source this file in the navigation terminal.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Use: source scripts/activate_g1_nav.sh" >&2
    exit 1
fi
_g1_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$_g1_root/.venv-g1-nav/bin/activate" ]]; then
    echo "Missing ROS 2 Humble or .venv-g1-nav; see README.md and run scripts/setup_g1_real.sh." >&2
    unset _g1_root
    return 1
fi
unset PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
source "$_g1_root/.venv-g1-nav/bin/activate"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
unset _g1_root
echo "G1 navigation: ROS 2 Humble, $(python --version 2>&1), ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
