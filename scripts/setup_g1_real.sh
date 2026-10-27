#!/usr/bin/env bash
# User-local ROS middleware and official messages; no sudo or simulator install.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 || "$(dpkg --print-architecture)" != amd64 ]]; then
    echo 'This pinned local middleware setup targets Ubuntu 22.04 amd64 / ROS 2 Humble.' >&2
    exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo 'ROS 2 Humble is required; see scripts/install_g1_nav_deps.sh.' >&2
    exit 1
fi
for program in git colcon cmake; do
    if ! command -v "$program" >/dev/null; then
        echo "Missing $program; run scripts/install_g1_nav_deps.sh first." >&2
        exit 1
    fi
done
if [[ ! -x .venv-g1-nav/bin/python ]]; then
    /usr/bin/python3 -m venv --system-site-packages --without-pip .venv-g1-nav
fi
mkdir -p .cache-g1/ros-cyclone/debs .cache-g1/ros-cyclone/root
packages=(
    'ros-humble-rmw-cyclonedds-cpp=1.3.5-1jammy.20260907.205235'
    'ros-humble-cyclonedds=0.10.5-2jammy.20260226.013234'
    'ros-humble-iceoryx-binding-c=2.0.5-1jammy.20260226.012512'
    'ros-humble-iceoryx-hoofs=2.0.5-1jammy.20260226.011054'
    'ros-humble-iceoryx-posh=2.0.5-1jammy.20260226.011738'
    'ros-humble-rosidl-generator-dds-idl=0.8.1-2jammy.20260605.104953'
)
for spec in "${packages[@]}"; do
    name="${spec%%=*}"
    version="${spec#*=}"
    deb=".cache-g1/ros-cyclone/debs/${name}_${version}_amd64.deb"
    if [[ ! -f "$deb" ]]; then
        (cd .cache-g1/ros-cyclone/debs && apt-get download "$spec")
    fi
    dpkg-deb -x "$deb" .cache-g1/ros-cyclone/root
done
revision=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
if [[ ! -d .cache-g1/unitree_ros2/.git ]]; then
    git clone --no-checkout https://github.com/unitreerobotics/unitree_ros2.git .cache-g1/unitree_ros2
    git -C .cache-g1/unitree_ros2 checkout --detach "$revision"
fi
if [[ "$(git -C .cache-g1/unitree_ros2 rev-parse HEAD)" != "$revision" ]]; then
    echo "Official messages must use pinned commit $revision; existing checkout left intact." >&2
    exit 1
fi
source scripts/activate_g1_real.sh
colcon --log-base .cache-g1/real-ros-build/log build \
    --base-paths .cache-g1/unitree_ros2/cyclonedds_ws/src/unitree/unitree_hg \
                 .cache-g1/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api \
    --build-base .cache-g1/real-ros-build/build \
    --install-base .cache-g1/real-ros-build/install --merge-install \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
echo 'Ready: ./run_g1_real.sh (motion output disabled by default)'
