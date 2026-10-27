# 当前主线验证记录

整理日期：2026-10-28。代码来自当前开发工作区，45 个导出源码／配置文件逐字节一致，
SHA-256 见 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)。

## 独立目录验证

| 检查 | 结果 |
| --- | --- |
| CPU 策略环境从空环境创建 | 成功；未借用 Isaac Sim / 原部署 venv |
| MuJoCo 环境从空环境创建 | 成功；未借用原仿真 venv |
| 固定提交 SDK、OpenHomie、官方锁腰模型下载 | 成功；ONNX／模型校验通过 |
| CPU 与 MuJoCo 的 pip 依赖一致性 | `pip check` 均通过 |
| ROS 中间件及官方消息安装／编译 | `unitree_api`、`unitree_hg` 成功 |
| 部署、速度链路、LowState 回环、速度门测试 | 46 项通过 |
| ROS 回环模拟端点 | 5 项通过，包含原厂 FSM 500/801 和 OpenHomie 放行／停机 |
| MuJoCo 模型、关节、执行与 viewer 隔离检查 | 5 项通过 |
| 当前降速范围的完整导航执行链 MuJoCo | 3 轮、459 秒、无跌倒 |
| 启动参数 | `--show-args` 成功；默认 homie、预览模式、禁止运动 |
| Python 编译、Shell 语法、Git 空白检查 | 通过 |

共 56 项自动化检查通过；本轮安装和验证未发送实机电机控制命令。
ROS 回环强制 Domain 119 和 127.0.0.1；LowState 接收回环强制 Domain 120 和 127.0.0.1。
策略部署测试在构造硬件接口前替换 SDK 发布器。

## 当前降速范围的跟踪结果

使用实际速度门、10 Hz UDP、50 Hz 补偿层、原 ONNX 和官方锁腰模型，500 Hz 物理仿真。
阶段末 4 秒的三轮均值如下；[原始摘要](mujoco-validation.json)。

| 导航目标 | 实测均值 |
| --- | --- |
| 前进 0.30 m/s | 0.300 m/s |
| 横移 +0.15 / −0.15 m/s | +0.150 / −0.151 m/s |
| 纯转向 +0.30 / −0.30 rad/s | +0.002 / −0.015 rad/s |
| 混合前进 0.15、横移 0.075、转向 0.15 | 0.149、0.074、0.161 |

纯转向跟踪未通过；前进和横移的阶段均值接近目标。当前 PI 补偿的策略输入上限为 0.6，
此前更高的补偿范围测试结果不能直接套用到当前配置。
实机仍需复测降速后的路径偏移；本轮整理没有调整训练权重或里程计来源。

## 复验命令

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH .venv-g1-deploy/bin/python -m unittest \
  scripts.test_g1_homie_deploy scripts.test_g1_homie_nav2 \
  scripts.test_g1_lowstate_reader scripts.test_g1_real_guards

source scripts/activate_g1_real.sh
python -m unittest scripts.test_g1_real_ros
ros2 launch navigation/g1_real_navigation.launch.py --show-args

env -u PYTHONPATH -u LD_LIBRARY_PATH OMP_NUM_THREADS=1 \
  .venv-g1-mujoco/bin/python -m unittest scripts.test_g1_mujoco
env -u PYTHONPATH -u LD_LIBRARY_PATH OMP_NUM_THREADS=1 \
  .venv-g1-mujoco/bin/python scripts/benchmark_g1_homie_nav2.py \
  --protocol deployment --trials 3 --output .cache-g1/portable-validation/mujoco
```

完整包版本见 [environment-versions.json](environment-versions.json)。
