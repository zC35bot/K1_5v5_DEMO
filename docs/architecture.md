# 项目总览

## 1. 仓库定位

这是一个基于 ROS 2 的 RoboCup 5v5 机器人项目，核心由三个运行节点组成：

- `vision`：相机图像推理、目标检测、线段分割、目标三维位置估计
- `brain`：状态汇总、定位、行为树决策、队友通信、机器人运动控制
- `game_controller`：接收裁判机 UDP 广播并转换成 ROS 2 话题

除此之外，仓库还包含若干接口包：

- `booster_ros2_interface`：机器人底层接口消息
- `robocup_ros2_interface/src/vision_interface`：视觉消息接口
- `robocup_ros2_interface/src/game_controller_interface`：裁判机消息接口
- `booster_msgs`：RPC 请求/响应消息

## 2. 目录结构

仓库里真正需要长期维护的内容主要集中在以下目录：

```text
configs/
  fastdds.xml                  DDS 通信配置

scripts/
  build*.sh                    构建脚本
  start*.sh                    启动脚本
  stop.sh                      停止脚本
  test_visual_kick.py          直接下发视觉踢球 RPC 的测试脚本

src/brain/
  behavior_trees/              行为树 XML
  config/config.yaml           Brain 参数
  launch/launch.py             Brain 启动入口
  src/*.cpp                    Brain 核心逻辑

src/vision/
  config/vision.yaml           Vision 参数
  config/field.yaml            偏移标定用场地点位
  launch/launch.py             Vision 启动入口
  src/vision_node.cpp          Vision 主节点
  src/calibration/*            标定流程
  model/*.engine               TensorRT 模型

src/game_controller/
  launch/launch.py             裁判机节点启动入口
  src/game_controller_node.cpp UDP -> ROS2 转换逻辑
```

## 3. 运行主链路

### 3.1 典型启动流程

最常用的总启动脚本是：

- `scripts/start.sh`

它做了几件事：

1. 杀掉旧的 `vision_node / brain_node / game_controller` 等进程。
2. `source ./install/setup.bash` 加载工作空间。
3. 启动 `vision`。
4. 启动 `brain`。
5. 启动 `game_controller`。

### 3.2 启动后数据流

系统主数据流大致如下：

1. 相机驱动向 `/boostercamera/head/rgb` 和 `/boostercamera/head/depth` 发图像。
2. `vision_node` 订阅图像和头部位姿 `/head_pose`。
3. `vision_node` 发布：
   - `/booster_vision/detection`
   - `/booster_vision/line_segments`
   - `/booster_vision/ball`
   - `/booster_vision/t_head2base`
4. `game_controller_node` 监听 UDP 3838，把裁判机数据转成 `/robocup/game_controller`。
5. `brain_node` 订阅视觉、裁判机、里程计、低层状态、头部位姿、跌倒恢复状态。
6. `brain_node` 在 100 Hz tick 中执行行为树，并通过 `RobotClient` 向 `LocoApiTopicReq` 发控制 RPC。

### 3.3 Brain 内部线程结构

`src/brain/src/main.cpp` 里实际跑了三部分：

- 主线程：`SingleThreadedExecutor`，负责 `brain_node` 自己的大部分 ROS 回调
- 100 Hz 后台线程：不断执行 `brain->tick()`
- 扩展订阅线程：单独订阅 `/remote_controller_state` 和 `/robocup/game_controller`

这意味着：

- 行为树 tick 和 ROS 回调是并发关系，不是单线程串行
- `BrainData` 里有些容器带互斥锁，有些标量没有锁，修改代码时要注意线程可见性和竞态

## 4. 各包职责简述

### 4.1 `vision`

负责：

- 目标检测
- 线段分割
- 基于颜色或深度的目标三维位置估计
- 视觉标定
- 保存原始数据用于离线分析

### 4.2 `brain`

负责：

- 聚合视觉、裁判机、遥控器、里程计和低层状态
- 维护机器人、球、障碍物、队友状态记忆
- 行为树决策
- 球场定位
- 队友通信
- 向机器人运动控制 RPC 发命令

### 4.3 `game_controller`

负责：

- 监听裁判机 UDP 广播
- 可选 IP 白名单过滤
- 将 RoboCup 标准数据结构展开成 ROS 2 消息

## 5. 配置覆盖关系

### 5.1 Brain

`brain` 的参数来源按优先级从低到高：

1. `src/brain/config/config.yaml`
2. 可选的 `src/brain/config/config_local.yaml`
3. `launch.py` 动态注入参数

其中 `launch.py` 目前会覆盖的关键参数有：

- `tree_file_path`
- `vision_config_path`
- `vision_config_local_path`
- `game.player_start_pos`，通过 `pos:=...`
- `game.player_role`，通过 `role:=...`
- `use_sim_time`，通过 `sim:=true`
- `rerunLog.enable_file / enable_tcp`，通过 `disable_log:=true`
- `enable_com`，通过 `disable_com:=true`

### 5.2 Vision

`vision` 的配置覆盖关系略特殊：

1. 启动参数指定的目录中的 `vision.yaml`
2. 同目录下可选的 `vision_local.yaml`
3. launch 参数覆盖运行时字段，如 `show_det`、`save_data`、模型路径、相机类型

`src/vision/launch/launch.py` 会把两个配置文件路径作为命令行参数传给 `vision_node`，然后再通过 ROS 参数覆盖以下字段：

- `offline_mode`
- `show_det`
- `show_seg`
- `save_data`
- `save_depth`
- `save_fps`
- `detection_model_path`
- `segmentation_model_path`
- `camera_type`

## 6. 常用脚本与适用场景

### 6.1 构建

- `scripts/build.sh`：普通构建
- `scripts/build_debug.sh`：带 `Debug` 符号构建
- `scripts/build_brain.sh`：只编译 `brain`

### 6.2 运行

- `scripts/start.sh`：完整比赛链路
- `scripts/start_brain.sh`：只启动 `brain`
- `scripts/start_vision.sh`：只启动 `vision`
- `scripts/start_game_controller.sh`：只启动裁判机桥接
- `scripts/start_joystick.sh`：启动手柄输入节点
- `scripts/start_realsense.sh`：启动 RealSense ROS 驱动

### 6.3 其他

- `scripts/stop.sh`：杀进程
- `scripts/test_visual_kick.py`：直接向 `LocoApiTopicReq` 发视觉踢球 RPC

## 7. 已知仓库现状与不一致点

这些点建议先知道：

1. `scripts/assist.sh` 使用 `tree:=assist`，但当前仓库没有 `assist.xml`。
2. `scripts/calibrate.sh` 和 `scripts/start_calibration.sh` 使用 `tree:=calibrate`，但当前仓库没有 `calibrate.xml`。
3. `vision` 默认脚本经常把 `vision_config_path` 指到 `/opt/booster`，这依赖部署机上存在外部配置，不是仓库内自包含。
4. `brain` 某些策略参数虽然出现在 `config.yaml` 里，但当前代码并没有真正消费，详见 [参数与修改注意事项](./configuration.md)。
5. 当前 `StrikerDecide` 可能生成 `safe_shoot` 决策，但行为树里没有 `decision == 'safe_shoot'` 的执行分支，这会导致该决策实际无法落地。
