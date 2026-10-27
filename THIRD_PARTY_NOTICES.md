# 来源与许可

- 集成代码提取自 [abizovnuralem/go2_omniverse](https://github.com/abizovnuralem/go2_omniverse)
  开发工作区，基础提交 `db18970`，保留 BSD-2-Clause [LICENSE](LICENSE)。
- Nav2 配置参考官方 Humble `nav2_bringup/params/nav2_params.yaml`（Apache-2.0），
  许可位于 `LICENSES/Apache-2.0.txt`。SLAM Toolbox 配置参考官方
  `config/mapper_params_online_async.yaml`，许可位于 `LICENSES/slam_toolbox-LICENSE`。
- OpenHomie 原始 ONNX、观测契约与部署参数来自
  [InternRobotics/OpenHomie](https://github.com/InternRobotics/OpenHomie)，固定提交
  `33071f79f1884bb122bd45631c8c60d61dac1eb8`。
  对应上游声明 CC BY-NC 4.0，保留 [原许可](LICENSES/OpenHomie-LICENSE)。
  本项目编写了 Python ONNX／MuJoCo／ROS 接口与速度补偿，未改训练权重。
  原始权重、源码和网格由安装脚本从上游获取，不随 Git 仓库提交。
- 官方 Python SDK（BSD-3-Clause）固定提交
  `814556d15970dd2ecf1c9984e845ca02ab07e206`，保留
  [SDK 许可](LICENSES/Unitree-SDK-BSD-3-Clause.txt)。
- 官方 ROS 消息来自 `unitreerobotics/unitree_ros2` 固定提交
  `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88`；官方锁腰模型来自
  `unitreerobotics/unitree_ros` 固定提交 `15ebe53f4cd77e7d745ae83b9d7ff90e685e8998`。
  第三方下载保留其上游许可，均缓存于 `.cache-g1/`。

本仓库不分发预训练权重、机器人网格、ROS 二进制或现场地图。
