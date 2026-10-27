# G1 实机建图与原厂步态导航

本仓库保留现场跑通的原厂步态导航链路。安装与常用命令见 [README](../README.md)。
启动入口是 `./run_g1_real.sh`，从仓库根目录执行，自动加载实机 ROS 环境。

## 模式与操作

| 启动命令 | RViz 的 2D Goal Pose | 速度输出 |
| --- | --- | --- |
| `./run_g1_real.sh` | 只规划预览 | 禁止 |
| `./run_g1_real.sh allow_motion:=true goal_mode:=navigate` | 执行 NavigateToPose | 还需手动放行 |
| 上述命令加 `rviz:=false` | 无 RViz 界面 | 与所选模式一致 |

已有入口时先退出旧入口；工作区运行锁阻止同目录重复启动。
不要同时从其他工作区启动另一套速度桥或低层控制程序。

执行模式下另开终端：

```bash
source scripts/activate_g1_real.sh
cat .cache-g1/real-runtime/status.json
ros2 service call /g1_real/enable_motion std_srvs/srv/SetBool "{data: true}"
```

只有状态新鲜且服务返回 `success: true` 后才发送执行目标。
RViz 的 Fixed Frame 为 `g1_real/map`；点击 **2D Goal Pose**，
在已知空闲区域按下鼠标并拖出朝向后松开。先用附近小目标验证方向、避障和停止。
不要在放行前预先发送执行目标，避免放行后继续跟踪未结束的旧目标。

解除放行会立即阻止后续速度，原先已放行时发送零速，并请求取消所有 NavigateToPose 目标：

```bash
ros2 service call /g1_real/enable_motion std_srvs/srv/SetBool "{data: false}"
```

取消服务不可用时桥仍保持禁止输出，响应会报告取消服务不可用。
重新放行前确认旧目标已结束，再选择新目标。现场紧急停止使用已确认有效的原厂遥控器停止方式。
正常完成后 Ctrl+C，或执行 `./run_g1_real.sh --stop`。

## 话题与接口

| 输入／输出 | 类型／用途 |
| --- | --- |
| `/utlidar/cloud_livox_mid360` | 原机 MID360 点云 |
| `/utlidar/imu_livox_mid360` | 雷达 IMU，只转发，未额外融合到 SLAM |
| `/dog_odom` | 原机状态估计里程计 |
| `/lowstate` | `unitree_hg/LowState`，机型和腰 yaw |
| `/g1_real/points`、`/g1_real/scan` | 校时后的点云和二维导航切片 |
| `/g1_real/map`、`/g1_real/odom` | SLAM 地图和转发里程计 |
| `/g1_real/preview_goal` | 只请求 ComputePathToPose |
| `/g1_real/goal_pose` | Nav2 NavigateToPose 执行目标 |
| `/g1_real/cmd_vel_nav` → `/g1_real/cmd_vel` | Nav2 速度经平滑后进入桥 |
| `/api/sport/request`、`/api/sport/response` | 原厂状态查询／速度 RPC |
| `/g1_real/status` | JSON 状态，同时保存为 `.cache-g1/real-runtime/status.json` |

桥遵循官方 [G1 LocoClient](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/g1/loco/g1_loco_client.py)
的协议，仅实现 GET_FSM_ID **7001** 和 SET_VELOCITY **7105**。
速度格式为 `{"velocity":[vx,vy,wz],"duration":0.25}`。
没有 LowCmd、Start、ReleaseMode 或自动原厂模式恢复操作。
RViz 执行模式使用 Nav2 Humble 内置的
[`goal_pose` → NavigateToPose 订阅](https://api.nav2.org/nav2-humble/html/navigate__to__pose_8cpp_source.html)。

## 放行条件与速度

放行要求：启动参数允许运动、机型 `mode_machine=6`、传感器及 FSM 数据新鲜、
FSM 在 `[500, 801]` 内、无时间／速度 RPC 故障。
500 来自官方 Start 实现；801 按现场固件读数和使用者要求兼容。
允许列表和数据新鲜度检查属于本项目集成逻辑，不是固件内置阈值。
当前现场到点成功时读数为 500；801 已完成门控回归测试。
可通过桥的 ROS 参数 `allowed_fsm_ids` 显式配置，桥不会改变机器人 FSM。

| 参数 | 值 |
| --- | --- |
| 前向／横向／角速度上限 | 0.15 m/s、0.06 m/s、0.25 rad/s |
| 速度变化率 | 0.20 m/s²、0.15 m/s²、0.40 rad/s² |
| 指令看门狗 | 0.35 s；超时输出零速 |
| 原厂速度请求 | 10 Hz；每条 duration=0.25 s |
| 传感器／FSM 断流、速度 RPC 故障 | 零速并解除放行；恢复后不会自动继续 |

Nav2 禁止规划穿越未知区，行为树不执行自动旋转或倒退恢复。
本地地图覆盖范围不等于可通行范围；先确认地图、地面和机器人位姿一致。
速度请求是否被接受看 RPC 回复，到点结果看 Nav2 的 NavigateToPose 状态。

## 坐标与时间

TF 链为 `g1_real/map → odom → base_footprint → base_link → waist_yaw → torso → lidar`。
使用官方锁腰 27 自由度 G1 名义 URDF 参数；腰 yaw 来自电机槽位 12。
torso 到雷达平移 `[0.0002835, 0.00003, 0.428434]` m，RPY `[π, 0.05112069, 0]`。
现场仍需核对 `/dog_odom` 的 robot_center 与 pelvis、地面高度及雷达实际安装外参。
桥提供 `odom_to_pelvis_z_offset`、`lidar_xyz`、`lidar_pitch` ROS 参数用于校正。

第一帧原机里程计建立固定时间偏移；点云和里程计共用偏移，保留传感器时间差，
并未实现 PTP/NTP。时间跳变或漂移会锁定故障，需要显式重启建立新映射。
全部节点使用墙钟，`use_sim_time=false`。

二维导航切片取地面以上 0.25–1.8 m、距离 0.5–20 m。
低障碍、负障碍和楼梯不在当前切片检测保证内。
IMU 外参未校准，该转发数据不参与 SLAM。

## 常见问题

**地址丢失／Address already assigned：**
`ip -brief -4 address` 检查实际接线网卡。用以下命令添加电脑端附加地址，先替换网卡名称：

```bash
nmcli device modify <robot-interface> +ipv4.addresses 192.168.123.222/24
```

地址已存在时无需重复添加；临时地址可能在断线、重启后丢失。
入口使用系统 `awk` 精确匹配地址，无需安装 ripgrep。

**`sensors_ready=true`、`fsm=null`：**
表示没有有效 FSM 回复，不等于 FSM 0。检查服务端：

```bash
source scripts/activate_g1_real.sh
ros2 topic info /api/sport/request
ros2 topic info /api/sport/response
```

现场曾出现 request 的 Subscription count 和 response 的 Publisher count 都为 0，
而原厂 CheckMode 返回 `{"form":"0","name":""}`。
当时原厂运动模式未恢复，但传感器仍正常发布。
应退出低层程序并按宇树官方流程恢复原厂控制，确认原厂遥控器正常后重启导航。

**`fsm=801`、旧版入口拒绝放行：**
早期代码仅允许 500。当前版本支持 `[500, 801]`；修改文件后需要重启入口才会加载。
查看状态中的 `allowed_fsm_ids` 和放行服务的具体拒绝原因。
0、1、4 及未知 FSM 不会因加入 801 而被放行。

**`command watchdog: stop`：**
表示没有新的速度指令，桥在保持零速；不代表放行失败。
执行模式下在 RViz 重新选择附近目标，检查规划结果和 Nav2 日志。
默认预览模式只产生青色预览路径，不会驱动机器人。

**旧地图还在但传感器不更新：**
检查状态中的数据年龄、`clock_fault` 和 `cloud_time_valid`。
换网卡／地址后重新启动入口；不能仅凭 RViz 留存画面判断链路正常。

## 保存与检查

当前入口运行在线 SLAM；保存地图不会自动切换到 AMCL，暂无旧地图定位启动入口。
可用默认禁止运动模式运行 `python scripts/check_g1_real.py`，
只计算附近路径、保存局部地图和报告至 `.cache-g1/validation/stationary/`。
它不会发送 NavigateToPose；若桥已允许运动，该静止验收检查不会判为通过。

离线测试：`python3 scripts/test_g1_real_guards.py`。
ROS 回归测试：激活实机环境后运行 `python scripts/test_g1_real_ros.py`，
脚本强制回环网卡及独立 Domain 119，以模拟服务验证 500／801 放行、非法 FSM 拒绝、
速度请求、零速解除放行、目标取消和 RViz 模式话题，不连接真实机器人。
