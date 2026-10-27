# 依赖与可复现安装

目标平台为 Ubuntu 22.04 amd64 / ROS 2 Humble / 系统 Python 3.10。
先安装 ROS 2 Humble 并配置 ROS apt 软件源，再运行 `scripts/install_g1_nav_deps.sh`。
该步骤通过系统 apt 安装 Nav2、SLAM Toolbox、pointcloud_to_laserscan、RViz、
colcon、CMake 及 Python YAML／NumPy／Pillow 等依赖，需要 sudo。
这些系统包使用 apt 软件源候选版本；下面的中间件和官方消息固定版本。
整理验证时 Nav2 为 1.1.20，SLAM Toolbox 为 2.6.10。

`scripts/setup_g1_real.sh` 不使用 sudo，创建 `.venv-g1-nav`，继承 ROS 所需的系统 Python 包，
不继承已有仿真 venv，不通过 pip 安装 ROS 包。

| 本地解包的 ROS 中间件包 | 固定 Debian 版本 |
| --- | --- |
| rmw-cyclonedds-cpp | `1.3.5-1jammy.20260907.205235` |
| cyclonedds | `0.10.5-2jammy.20260226.013234` |
| iceoryx-binding-c | `2.0.5-1jammy.20260226.012512` |
| iceoryx-hoofs | `2.0.5-1jammy.20260226.011054` |
| iceoryx-posh | `2.0.5-1jammy.20260226.011738` |
| rosidl-generator-dds-idl | `0.8.1-2jammy.20260605.104953` |

下载到 `.cache-g1/ros-cyclone/debs/`，解包到 `.cache-g1/ros-cyclone/root/`。
如上游 apt 源移除固定版本，安装会明确失败；应使用对应版本的缓存包或核实兼容版本后更新脚本，
不应默默替换中间件。该脚本当前不支持 arm64。

官方消息来自 `unitreerobotics/unitree_ros2` 提交
`668d1ec5a05d1c38d3306bdca7d59f2ba3581a88`。
源码放在 `.cache-g1/unitree_ros2`，编译／安装至 `.cache-g1/real-ros-build/`。
只编译 `unitree_hg` 和 `unitree_api`，不编译官方运动示例。

环境入口 `source scripts/activate_g1_real.sh` 设置：

- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，加载本地中间件和消息 overlay。
- `ROS_DOMAIN_ID=0`，可由 `G1_REAL_ROS_DOMAIN_ID` 显式覆盖。
- `G1_ROBOT_ADDRESS=192.168.123.222`，可替换为电脑端机器人子网地址。
- `CYCLONEDDS_URI` 绑定该地址，允许机器人子网 DDS 发现。

因此不需要手动激活仿真环境，也不需要安装 Python Unitree SDK。
首次配置后，后续启动和离线测试使用本地缓存即可。
