# G1 Nav2：实机导航与 OpenHomie

宇树 G1 官方锁腰 27 自由度机型，真实 MID360 雷达与原机里程计，RViz 选点导航。
当前主线默认使用 **OpenHomie 原权重＋速度闭环补偿**；可切回原厂行走控制。

```text
MID360 / dog_odom / LowState → TF → LaserScan → SLAM Toolbox → Nav2 → 速度平滑与放行
    ├─ homie（默认）→ 速度 PI / 转向前馈 → ONNX → 官方 SDK LowCmd
    └─ factory → 官方速度 API → 原厂行走控制
```

当前导航目标上限：前进 **0.30 m/s**、横移 **±0.15 m/s**、转向 **±0.30 rad/s**。
加减速度为 `[0.2, 0.15, 0.4]`。补偿后策略输入范围保留 `±[0.6,0.6,0.6]`。
原厂链路已由使用者确认选点成功；OpenHomie 实机仍存在轨迹偏移，近期已降速。
0.6 策略输入上限下的纯转向在 MuJoCo 跟踪较差；混合行走转向可跟踪。详见 [验证与已知问题](docs/VALIDATION.md)。

## 快速安装

目标：**Ubuntu 22.04 amd64、ROS 2 Humble、Python 3.10**。
先安装 ROS 2 Humble 并配置 ROS apt 软件源，然后执行：

```bash
git clone https://github.com/wzhou0804/g1_nav2.git
cd g1_nav2
bash scripts/install_g1_nav_deps.sh
bash scripts/setup_g1_real.sh
bash scripts/setup_g1_homie.sh       # 默认 OpenHomie 执行端；使用原厂时可跳过
```

独立创建 ROS 和 CPU 推理环境；不需要 Isaac Sim、CUDA 或 PyTorch。
安装时按固定提交下载 SDK、原始 ONNX 和模型并检查校验和，均放在 `.cache-g1/`。
仅安装环境不会连接机器人或启动电机控制。[依赖版本](docs/DEPENDENCIES.md)。

## 网络与启动

把下方网卡名换为本机连接 G1 的有线网卡；地址是**电脑端地址**：

```bash
export G1_ROBOT_INTERFACE=enx207bd298e838
export G1_ROBOT_ADDRESS=192.168.123.222
ip -brief -4 address
# 已有该地址则跳过；使用 NetworkManager 为当前连接添加地址
nmcli device modify "$G1_ROBOT_INTERFACE" +ipv4.addresses "$G1_ROBOT_ADDRESS/24"
```

默认只建图／预览规划，不允许导航运动：

```bash
./run_g1_real.sh
```

执行导航，终端 A：

```bash
./run_g1_real.sh allow_motion:=true goal_mode:=navigate
# 使用原厂行走控制时改为：
# ./run_g1_real.sh locomotion:=factory allow_motion:=true goal_mode:=navigate
```

OpenHomie 适用于腰 roll/pitch 已机械固定的本机型号；启动前先确认原厂站立状态和现场停止手段。
程序先只读预热，出现 `waiting_nav2_enable` 时仍未接管原厂控制。
终端 B，在同一仓库根目录执行：

```bash
source scripts/activate_g1_real.sh
cat .cache-g1/real-runtime/status.json
ros2 service call /g1_real/enable_motion std_srvs/srv/SetBool "{data: true}"
```

OpenHomie 随后自动检查／释放原厂控制、平滑进入默认姿态。看到 **`homie_active`** 后，
在 RViz 的 **2D Goal Pose** 点击空闲区域并拖出目标朝向。不需要额外启动策略或终端输入速度。
原厂执行端不接管 LowCmd，要求原厂控制和兼容 FSM 已就绪。

取消目标并停止导航，保持策略运行：

```bash
ros2 service call /g1_real/enable_motion std_srvs/srv/SetBool "{data: false}"
```

`Select`、Ctrl+C 或 `./run_g1_real.sh --stop` 会结束 OpenHomie 并进入阻尼，释放位置支撑；
退出前由外部支撑承重，恢复原厂控制使用官方流程。

## MuJoCo 离线验证（可选）

先完成 `setup_g1_homie.sh` 下载权重和模型，再执行：

```bash
bash scripts/setup_g1_mujoco.sh
MUJOCO_GL=glfw ./run_g1_mujoco.sh --viewer --feedback closed-loop --trials 1
# 导航速度门＋UDP＋补偿层＋ONNX 的无界面验证
OMP_NUM_THREADS=1 .venv-g1-mujoco/bin/python scripts/benchmark_g1_homie_nav2.py \
  --protocol deployment --trials 3 --output .cache-g1/nav2-check
```

该入口不连接机器人。运行结果、转换模型及视频均写入 `.cache-g1/`。

## 文档与目录

- [实机操作与排错](docs/G1_REAL_NAVIGATION_ZH.md)
- [依赖与固定版本](docs/DEPENDENCIES.md)
- [验证记录与复测命令](docs/VALIDATION.md)
- [代码来源与许可](THIRD_PARTY_NOTICES.md)
- `navigation/`：传感器、TF、SLAM/Nav2 配置、速度闭环与行走接口。
- `scripts/`：安装、资产下载、部署、隔离测试与 MuJoCo 验证。

不提交虚拟环境、权重、第三方源码、现场地图、日志及旧实验报告。
`student_deploy_core.py` / `deploy_g1_student.py` 等名称沿用开发工作区的共用实现；
本主线入口选择 OpenHomie，不安装旧 HTD Student 策略。
