# G1 实机导航操作

从 [README](../README.md) 完成安装、网卡配置后，在仓库根目录运行所有命令。
当前主线使用原机 MID360、`/dog_odom`、`/lowstate`，不使用相机或仿真真值。

## 模式与启动现象

G1 `mode_machine=6` 是锁腰 27 自由度机型的通信标识，不是遥控器需要切入的“模式 6”。
腰 roll/pitch 已机械固定，腰 yaw 和双臂保持 OpenHomie 默认零位关节目标。

```bash
./run_g1_real.sh allow_motion:=true goal_mode:=navigate
```

| 日志阶段 | 行为 |
| --- | --- |
| `waiting_lowstate / preflight_shadow` | 接收状态、检查数据、只读推理预热 |
| `waiting_nav2_enable` | 等待服务放行，原厂控制仍保留 |
| `checking_original_mode / releasing_original_mode` | 放行后检查并释放原厂运动控制；已处于调试空模式时不重复释放 |
| `moving_to_default` | 按关节位移平滑进入默认姿态，双臂也会过渡 |
| `homie_active` | 策略已运行，可在 RViz 选点 |
| `exit_damping` | 策略退出，停止位置保持，不自动恢复原厂控制 |

服务操作：

```bash
source scripts/activate_g1_real.sh
ros2 service call /g1_real/enable_motion std_srvs/srv/SetBool "{data: true}"
# 停止行走、取消当前目标，策略继续运行
ros2 service call /g1_real/enable_motion std_srvs/srv/SetBool "{data: false}"
```

Nav2 模式不需要 L1+Start / L1+A；启动遥控器控制的独立策略入口时才使用该握手。
未放行时可用原厂遥控器建图。地图使用在线 SLAM；当前没有单独 AMCL 定位入口。
RViz Fixed Frame 为 `g1_real/map`，执行目标使用 **2D Goal Pose**。

## 原厂执行端

```bash
./run_g1_real.sh locomotion:=factory allow_motion:=true goal_mode:=navigate
```

其雷达、里程计、SLAM、Nav2 与服务接口相同，速度经官方 API 7105 下发。
原厂 FSM 支持当前已测试的 500 / 801；不会替操作员切换模式。此前运行低层策略时，先按官方流程恢复原厂控制。

## 速度与反馈

| 项目 | 当前值 |
| --- | --- |
| 前进目标上限 | 0.30 m/s |
| 横移目标范围 | ±0.15 m/s |
| 转向目标范围 | ±0.30 rad/s |
| 合成平移目标上限 | 0.30 m/s |
| 前进／横移／转向加减速度 | 0.2 / 0.15 / 0.4 |
| 补偿后策略输入范围 | ±[0.6, 0.6, 0.6] |
| 转向前馈 | 0.4 rad/s |
| Nav2 链路 / 策略推理 / LowCmd | 10 / 50 / 500 Hz |

目标上限需同时修改 `g1_real_navigation.launch.py` 中 DWB、velocity_smoother 与 `real_g1_core.py` 的速度门。
补偿输入范围在 `scripts/deploy_g1_student.py`，与机器人目标速度分别配置。
反馈来自原机 `/dog_odom` 的机体速度，按姿态投影到水平朝向坐标系；不修改 SLAM 使用的里程计。
普通到点停车逐步减速；解除放行、通信失效或指令看门狗触发时立即送零。
修改配置后重新启动生效。

## 状态与排错

```bash
cat .cache-g1/real-runtime/status.json
cat .cache-g1/homie-deploy/nav2/status.json
tail -n 1 .cache-g1/homie-deploy/nav2/telemetry.jsonl
```

- `armed=false`：尚未服务放行，或发生通信／传感器故障后取消放行。恢复后显式重新放行。
- `motion_ready=false`：查看 `motion_reason`、`homie_status.phase` 与 `last_disarm`。
- `OpenHomie unavailable / ack_age`：本机策略应答超过 0.6 秒。先看策略是否已经退出及其 `reason`。
- `LowState stale or missing`：100 ms 内没有有效新 tick。查看 `lowstate_receive_at_stop` 中的
  `last_message_age_ms` 与 `unique_tick_age_ms`；前者变大是接收空档，只有后者变大是重复 tick。
- `No valid trajectories / Failed to make progress`：局部控制器无法得到可执行轨迹或实际运动不足，
  检查障碍、地图与里程计，并对照 `desired_velocity / measured_velocity / policy_velocity`。
- `Address already assigned`：已有电脑端地址，无需重复添加。

SDK LowState 使用独立线程与持久 WaitSet 持续读取，保留 CRC、tick、机型与有效性检查。
自定义 50 ms 推理退出、100 ms 策略目标年龄退出、倾角／温度阈值和 PD 预算投影已在开发中移除。
LowState 100 ms、Nav2 指令 0.35 s、里程计 0.5 s、策略应答 0.6 s 属于当前接入逻辑的通信检查。

## 保存地图

```bash
source scripts/activate_g1_real.sh
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/g1_map" --ros-args \
  -r map:=/g1_real/map -p use_sim_time:=false
```

传感器使用常量时钟偏移映射到 ROS 墙钟，保留传感器之间的时间差。
TF 中的 `robot_center / pelvis` 关系与地面参考使用名义 URDF，换机器人后应核对安装标定。
