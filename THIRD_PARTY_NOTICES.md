# 来源与许可

- 本项目从 [abizovnuralem/go2_omniverse](https://github.com/abizovnuralem/go2_omniverse)
  开发工作区提取 G1 实机导航集成；保留原仓库 BSD-2-Clause [LICENSE](LICENSE)。
  提取时原仓库基础提交为 `db18970`，实机导航为其后开发的独立模块。
- `navigation/config/nav2.yaml` 基于 Nav2 Humble 的
  [`nav2_bringup/params/nav2_params.yaml`](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)
  修改命名空间、墙钟、速度、代价地图和行为配置。Nav2 为 Apache-2.0，
  保留相应许可文本于 `LICENSES/Apache-2.0.txt`。
- `navigation/config/slam.yaml` 基于 SLAM Toolbox Humble 的
  [`config/mapper_params_online_async.yaml`](https://github.com/SteveMacenski/slam_toolbox/blob/humble/config/mapper_params_online_async.yaml)
  修改命名空间、墙钟、传感器范围和建图参数；该项目声明 LGPL，见 `LICENSES/slam_toolbox-LICENSE`。
- 安装脚本从官方 [unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
  固定提交下载并编译 `unitree_api`、`unitree_hg` 消息。第三方源码／二进制不随本仓库分发，
  缓存在被忽略的 `.cache-g1/` 中；各依赖保留其上游许可。

本仓库不分发预训练策略、机器人模型资产、ROS 二进制或现场地图。
