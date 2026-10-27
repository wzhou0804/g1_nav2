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

使用原厂执行端时无需 Python Unitree SDK。默认 OpenHomie 通过下方独立环境安装 SDK。
首次配置后，后续启动和离线测试使用本地缓存即可。

## OpenHomie CPU 策略环境

`bash scripts/setup_g1_homie.sh` 创建 `.venv-g1-deploy`，不继承 ROS 或 Isaac Sim。
`create_g1_venv.sh` 用系统 Python/pip 和固定 `pip==25.2` wheel 引导，无需旧仿真 venv。
系统 pip 由 `install_g1_nav_deps.sh` 安装。

| 依赖 | 固定版本／提交 |
| --- | --- |
| NumPy / PyYAML | 1.26.4 / 6.0.3 |
| ONNX Runtime CPU | 1.23.2 |
| Python CycloneDDS | 0.10.2，独立于 ROS 的 0.10.5 |
| OpenCV（官方 SDK 元数据依赖） | 4.11.0.86 |
| 官方 Python SDK | `814556d15970dd2ecf1c9984e845ca02ab07e206` |
| OpenHomie | `33071f79f1884bb122bd45631c8c60d61dac1eb8` |
| 官方锁腰 URDF／网格 | `15ebe53f4cd77e7d745ae83b9d7ff90e685e8998` |
| 原始 ONNX SHA-256 | `afe6363c83c8b9ab8b1a2fed598fce7de35ff045eee1a95b336fd6720e3d3d49` |

安装器下载 SDK、OpenHomie 与官方模型至 `.cache-g1/`；不安装 PyTorch，也不编译 SDK 的运动示例。
`run_g1_homie.sh` 清理 ROS/Isaac 路径后调用独立 CPU Python，内部复用 `run_g1_student.sh`。

## MuJoCo（可选）

`bash scripts/setup_g1_mujoco.sh` 创建 `.venv-g1-mujoco`。
主要版本：MuJoCo 3.2.3、ONNX Runtime 1.23.2、NumPy 1.26.4、Matplotlib 3.10.3。
完整直接依赖见根目录 `requirements-g1-mujoco.txt`；运行不需要 ROS、DDS 或物理机器人。
先运行 OpenHomie 安装器获取策略与模型，再执行 MuJoCo 测试。
主线安装验证时的完整 pip 包清单见 `docs/environment-versions.json`；它记录传递依赖，
直接安装约束分别在两个 requirements 文件中。
