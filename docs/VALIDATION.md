# 整理验证记录

日期：2026-10-27。

现场 RViz 选点导航成功由使用者确认。独立仓库在新目录执行以下验证：

| 检查 | 结果 |
| --- | --- |
| 独立 Python 环境与固定版本中间件下载 | 成功 |
| 官方消息从固定提交下载、编译 | `unitree_api`、`unitree_hg` 均成功 |
| 速度门控、通信故障、时间映射测试 | 7 项通过 |
| 回环 ROS 模拟端点测试 | 3 项通过，包含 FSM 500／801、拒绝非法状态、速度与取消流程 |
| 启动参数检查 | `--show-args` 成功，预览模式为默认值 |
| Nav2、SLAM 运行参数比较 | 与现场工作区生成配置一致；行为树路径随新目录变化 |
| 速度桥、核心逻辑、RViz、行为树、启动脚本比较 | 与提取时的工作区源文件一致 |
| Python 编译、Shell 语法 | 通过 |

回环测试将 DDS 强制设为 Domain 119、127.0.0.1，使用模拟原厂服务和 Nav2 取消服务。
安装、配置比较和启动参数检查均未启动实机导航节点。

可复验命令：

```bash
python3 scripts/test_g1_real_guards.py
source scripts/activate_g1_real.sh
python scripts/test_g1_real_ros.py
ros2 launch navigation/g1_real_navigation.launch.py --show-args
```
