# G1 Nav2 实机导航

使用宇树 G1 **原厂行走控制**，接入真实 MID360 雷达和原机里程计，通过 ROS 2 Nav2 在 RViz 选点导航。
本项目从 `go2_omniverse` 开发工作区提取，只包含实机导航链路。

```text
MID360 + 原机里程计 + LowState
    → 时间映射与 TF → 点云转 LaserScan → SLAM Toolbox
    → Nav2 → 速度平滑 → 显式放行 → 官方速度接口 → 原厂行走控制
```

2026-10-27，使用者已确认现场 RViz 选点导航成功。整理后的版本检查配置等价性，
并提供 10 项离线／回环 ROS 测试；未在整理过程中重新驱动机器人。
项目不包含 Isaac Sim、MuJoCo、OpenHomie、Student 权重、现场地图或运行缓存。

## 环境安装

目标环境：Ubuntu 22.04 **amd64**、ROS 2 Humble、Python 3.10。
先按 [ROS 官方文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)安装 ROS 2 Humble，
并配置 ROS apt 软件源，然后执行：

```bash
git clone https://github.com/wzhou0804/g1_nav2.git
cd g1_nav2
bash scripts/install_g1_nav_deps.sh
bash scripts/setup_g1_real.sh
```

安装脚本创建独立 `.venv-g1-nav`，下载固定版本的本地 CycloneDDS ROS 中间件，
并从固定提交编译官方 `unitree_api`、`unitree_hg` 消息。无需安装仿真器、CUDA、PyTorch 或 Python Unitree SDK。
首次安装需要网络；详细版本见 [依赖记录](docs/DEPENDENCIES.md)。

## 网络与原厂控制

机器人通过有线网络连接电脑。电脑网卡需要有机器人子网的本地地址，例如 `192.168.123.222/24`：

```bash
ip -brief -4 address
# 把 <robot-interface> 换成连接机器人的网卡；已有该地址时跳过
nmcli device modify <robot-interface> +ipv4.addresses 192.168.123.222/24
```

这里的地址是**电脑端地址**，不要填机器人 IP。临时地址可能在重连／重启后消失。
其他电脑地址可用 `export G1_ROBOT_ADDRESS=192.168.123.xxx` 指定。
原机 DDS Domain 默认 0。

如果此前运行过低层策略，先退出低层程序，并按宇树官方流程恢复原厂控制。
现场确认落地、原厂遥控器可以正常站立和行走，再放行导航速度。
桥不会自动切换原厂模式，不发布 LowCmd。

## 启动与选点

默认只建图和预览规划，禁止速度输出：

```bash
./run_g1_real.sh
```

执行选点导航时，在终端 A 启动（已有入口时先退出，避免重复运行）：

```bash
./run_g1_real.sh allow_motion:=true goal_mode:=navigate
```

终端 B，在仓库根目录运行：

```bash
source scripts/activate_g1_real.sh
cat .cache-g1/real-runtime/status.json
```

确认 `sensors_ready`、`motion_ready` 都为 `true`，时间和 RPC 无故障。
`allow_motion:=true` 仅允许手动放行，初始 `armed` 仍为 `false`。
准备好后执行：

```bash
ros2 service call /g1_real/enable_motion std_srvs/srv/SetBool "{data: true}"
```

收到 `success: true` 后，在 RViz 保持 Fixed Frame 为 `g1_real/map`，
点击 **2D Goal Pose**，在附近已知空闲区域点击并拖出目标朝向。
绿色路径为 Nav2 规划路径；机器人由原厂控制器执行速度指令。
当前入口使用在线 SLAM，可先在未放行状态下用原厂遥控器建图。

**停止导航、取消目标并解除放行：**

```bash
ros2 service call /g1_real/enable_motion std_srvs/srv/SetBool "{data: false}"
```

完成后在终端 A 按 Ctrl+C，或执行 `./run_g1_real.sh --stop`。
详细操作、坐标标定、接口和排错见 [实机部署说明](docs/G1_REAL_NAVIGATION_ZH.md)。

## 保存地图与验证

```bash
source scripts/activate_g1_real.sh
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/g1_map" --ros-args \
  -r map:=/g1_real/map -p use_sim_time:=false

# 离线逻辑测试，不连接机器人
python3 scripts/test_g1_real_guards.py
# 回环 ROS 测试：脚本强制 Domain 119、127.0.0.1，不连接机器人
python scripts/test_g1_real_ros.py
```

`python scripts/check_g1_real.py` 用于默认禁止运动模式下的实机静止链路检查，
只请求规划路径，不发送导航执行目标；报告和地图写入 `.cache-g1/validation/stationary/`。
当前未提供保存地图后的 AMCL 定位启动入口。

## 目录

| 路径 | 用途 |
| --- | --- |
| `run_g1_real.sh` | 启动／停止入口、地址检查、工作区运行锁 |
| `navigation/real_g1.py` | 传感器、TF、原厂状态查询与速度桥 |
| `navigation/real_g1_core.py` | 时间映射、速度放行及通信看门狗 |
| `navigation/real_g1_preview.py` | RViz 只规划预览 |
| `navigation/config/` | 实机 Nav2、SLAM、RViz、行为树配置 |
| `scripts/` | 依赖安装、环境激活、诊断、隔离测试 |
| `docs/` | 部署与依赖说明 |

来源和许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
