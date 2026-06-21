# Brain 模块说明

如果当前目标是 15 天内快速做出守门员队长、3v3 传球、二过一以及守门员动态防守，而不是继续加复杂模型，建议先读：

- [Brain 15 天冲刺规划](./brain_fast_iteration_plan.md)

## 1. 入口与初始化

Brain 的入口在：

- `src/brain/src/main.cpp`

主流程如下：

1. `rclcpp::init`
2. 构造 `Brain`
3. `brain->init()`
4. 开一个 100 Hz 线程持续执行 `brain->tick()`
5. 再起一个独立 ROS context 订阅手柄和裁判机
6. 主 executor 处理 `brain_node` 的其他回调

`Brain::init()` 在 `src/brain/src/brain.cpp` 中完成以下对象初始化：

- `BrainConfig`
- `BrainData`
- `Locator`
- `BrainLog`
- `BrainTree`
- `RobotClient`
- `BrainCommunication`

然后创建订阅与发布。

## 2. 核心对象职责

### 2.1 `BrainConfig`

文件：

- `src/brain/include/brain_config.h`
- `src/brain/src/brain_config.cpp`

职责：

- 保存静态配置
- 解析球场尺寸
- 构造球场理论线段和理论标志点
- 对关键参数做合法性检查

`BrainConfig::handle()` 会校验：

- `player_start_pos` 必须是 `left/right`
- `player_role` 必须是 `striker/goal_keeper`
- `player_id` 必须在合法范围内
- `field_type` 必须是 `adult_size / kid_size / robo_league`

### 2.2 `BrainData`

文件：

- `src/brain/include/brain_data.h`
- `src/brain/src/brain_data.cpp`

职责：

- 保存运行态状态
- 维护机器人、球、障碍物、标志点、场线、队友通信信息
- 保存比赛状态、记忆状态、恢复状态
- 提供坐标变换辅助函数

这里的数据是行为树、视觉回调和通信线程共同读写的核心共享区。

### 2.3 `BrainTree`

文件：

- `src/brain/include/brain_tree.h`
- `src/brain/src/brain_tree.cpp`

职责：

- 注册行为树节点
- 加载 XML
- 初始化 blackboard
- 每 tick 调用 `tree.tickOnce()`

### 2.4 `RobotClient`

文件：

- `src/brain/include/robot_client.h`
- `src/brain/src/robot_client.cpp`

职责：

- 对底层运动 API 做一层封装
- 将行为树里的移动、抬头、踢球、起身等动作转成 RPC 请求

它最终通过发布 `booster_msgs::msg::RpcReqMsg` 到 `LocoApiTopicReq` 控制机器人。

### 2.5 `BrainCommunication`

文件：

- `src/brain/include/brain_communication.h`
- `src/brain/src/brain_communication.cpp`

职责：

- 向裁判机发送存活回包
- 通过广播发现队友
- 通过单播发送和接收队友状态

端口规则：

- 发现广播端口：`20000 + teamId`
- 队友状态单播端口：`30000 + teamId`

## 3. ROS 订阅与发布

### 3.1 主要订阅

`Brain::init()` 中创建的关键订阅：

- `/booster_vision/detection`
- `/booster_vision/line_segments`
- `/odometer_state`
- `/low_state`
- `/head_pose`
- `fall_down_recovery_state`
- 日志开启时还会订阅 `vision.image_topic`
- 总是订阅 `vision.depth_image_topic`

扩展线程还会订阅：

- `/remote_controller_state`
- `/robocup/game_controller`

### 3.2 主要发布

- `/play_sound`
- `/speak`
- `/kick_ball`
- `LocoApiTopicReq` 由 `RobotClient` 发布

另外，`brain` 自己也会广播 `odom -> base_link` 的 TF。

## 4. 100 Hz tick 主循环

`Brain::tick()` 的顺序非常重要：

1. `logLags()`
2. `logStatusToConsole()`
3. `updateLogFile()`
4. `updateMemory()`
5. `updateBallPrediction()`
6. `handleSpecialStates()`
7. `handleCooperation()`
8. `pubKickMsg()`
9. `tree->tick()`
10. `logDebugInfo()`
11. `statusReport()`
12. `playSoundForFun()`

这意味着行为树每次执行前，`BrainData` 里的记忆状态、比赛派生状态和协作状态都先被更新。

## 5. 回调逻辑

### 5.1 手柄回调 `joystickCallback`

文件位置：

- `src/brain/src/brain.cpp`

主要逻辑：

- 摇杆有输入时，`go_manual = true`
- `LT + X`：`control_state = 1`
- `LT + A`：`control_state = 2`，并把 `odom_calibrated = false`
- `LT + B`：`control_state = 3`
- `LT + Y`：切换前锋/守门员角色
- `LB`：`assist_chase = true`
- `RB`：`assist_kick = true`
- `LT + 方向键`：在线调 `vxFactor` 和 `yawOffset`

`control_state` 在 `game.xml` 里直接控制行为树的顶层分支。

### 5.2 裁判机回调 `gameControlCallback`

作用：

- 更新 `gc_game_state`
- 更新 `gc_game_sub_state_type`
- 更新 `gc_game_sub_state`
- 更新 `gc_is_kickoff_side`
- 更新 `gc_is_sub_state_kickoff_side`
- 更新 `gc_is_under_penalty`
- 更新比分和双方在场人数

状态映射关系：

- 一级状态：`INITIAL / READY / SET / PLAY / END`
- 二级状态：当前代码里把多种定球状态统一处理成 `FREE_KICK`

注意：

- 一旦本机进入罚下状态，会把 `odom_calibrated` 置回 `false`
- `secondary_state` 的多个具体类型被折叠为同一个 `FREE_KICK`，具体类型保存在 `data->realGameSubState`

### 5.3 视觉检测回调 `detectionsCallback`

主要流程：

1. 把 `vision_interface::msg::Detections` 转成 `GameObject`
2. 按类别分成球、门柱、人、对手、标志点
3. 分别调用：
   - `detectProcessBalls`
   - `detectProcessGoalposts`
   - `detectProcessMarkings`
   - `detectProcessRobots`
4. 更新视场框
5. 打 rerun log

### 5.4 场线回调 `fieldLineCallback`

作用：

- 把 `/booster_vision/line_segments` 里的 2D/3D 线段转成 `FieldLine`
- 用当前里程计姿态投到球场坐标系
- 再做线段归类与过滤

场线用于定位和出界判断。

### 5.5 里程计回调 `odometerCallback`

作用：

- 更新 `robotPoseToOdom`
- 结合 `odomToField` 计算 `robotPoseToField`
- 发布 `odom -> base_link` TF
- 在 rerun 里画当前机器人姿态

### 5.6 低层状态回调 `lowStateCallback`

当前主要使用：

- 头部 `yaw`
- 头部 `pitch`

后续很多视觉与找球逻辑依赖这两个量。

### 5.7 恢复状态回调 `recoveryStateCallback`

用于同步跌倒/起身状态，配合行为树中的 `CheckAndStandUp` 节点控制自动起身。

## 6. 记忆与派生状态

### 6.1 `updateMemory`

会依次调用：

- `updateBallMemory()`
- `updateRobotMemory()`
- `updateObstacleMemory()`
- `updateKickoffMemory()`

### 6.2 球记忆 `updateBallMemory`

逻辑：

- 根据 `strategy.ball_memory_timeout` 判断球记忆是否失效
- 更新球和队友球的相对位置
- 更新 blackboard 中的 `ball_range`

### 6.3 障碍物记忆 `updateObstacleMemory`

逻辑：

- 根据 `obstacle_avoidance.obstacle_memory_msecs` 清理超时障碍物
- 比赛 `READY` 状态或任意球摆位时，会把当前球也加入障碍物集合，用于避让

### 6.4 开球等待 `updateKickoffMemory`

作用：

- 当处于对方开球等待期时，设置 `wait_for_opponent_kickoff = true`
- 对方开球后或超时后解除等待

这直接影响前锋和守门员比赛分支是否允许行动。

## 7. 多机协作

核心函数：

- `handleCooperation()`
- `updateCostToKick()`
- `isPrimaryStriker()`

### 7.1 控球成本 `updateCostToKick`

成本主要由以下因素组成：

- 最近一次看到球的时间
- 球是否已知
- 球距离
- 球偏角
- 前方障碍物
- 与队友潜在冲突
- 当前是否倒地
- 当前是否已完成定位

然后用平滑方式更新到 `data->tmMyCost`。

### 7.2 主前锋判定

`isPrimaryStriker()` 的逻辑比较简单：

- 自己不是 `striker`，直接不是主前锋
- 没开队友通信时，自己默认就是主前锋
- 开通信后，如果存在 ID 更小且存活的前锋，则自己不是主前锋

这是一个相对保守但不复杂的判定方式。

## 8. 行为树结构

### 8.1 主树入口

文件：

- `src/brain/behavior_trees/game.xml`

主树按 `control_state` 切三种顶层模式：

- `1`：手动/辅助
- `2`：重定位模式
- `3`：正式自动比赛模式

### 8.2 自动比赛主分支

自动模式下又按比赛状态切：

- 罚下状态
- `TIMEOUT`
- 普通比赛 `NONE`
- 任意球 `FREE_KICK`

普通比赛 `NONE` 再按：

- `INITIAL`
- `READY`
- `SET`
- `PLAY`
- `END`

### 8.3 前锋比赛子树

文件：

- `src/brain/behavior_trees/subtrees/subtree_striker_play.xml`

流程大致是：

1. 自定位
2. 定位修正
3. 若等待对方开球，则仅看球并原地等待
4. 若球出界，执行回场逻辑
5. 否则：
   - 相机找球/跟球
   - `CalcKickDir`
   - `StrikerDecide`
   - 根据决策执行 `FindBall / Assist / Chase / RLVisionKick / Adjust / Kick`

### 8.4 守门员比赛子树

文件：

- `src/brain/behavior_trees/subtrees/subtree_goal_keeper_play.xml`

主要决策：

- `find`
- `intercept`
- `retreat`
- `chase`
- `adjust`
- `kick`

当前守门员比赛树已经把 `Intercept` 接入 [src/brain/behavior_trees/subtrees/subtree_goal_keeper_play.xml](../src/brain/behavior_trees/subtrees/subtree_goal_keeper_play.xml:38)。
`GoalieDecide` 会结合 `ball_will_breach`、预测拦截时间和球场位置切入 `intercept`，实现位置见 [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1111)。
当 `goalie_mode == guard` 时更多是站位与封堵；`goalie_mode == attack` 时会主动追球、拦截或出击。

### 8.5 定位子树

文件：

- `src/brain/behavior_trees/subtrees/subtree_locate.xml`

当前是组合定位，按顺序尝试：

- `SelfLocate`
- `SelfLocate1M`
- `SelfLocate2T`
- `SelfLocatePT`
- `SelfLocateLT`
- `SelfLocate2X`
- `SelfLocateBorder`

## 9. 关键决策节点

### 9.1 `StrikerDecide`

作用：

- 综合球位置、偏角、是否主控、是否允许自动视觉踢球、是否避障等条件
- 输出 `decision`

当前可能输出：

- `find`
- `assist`
- `chase`
- `auto_visual_kick`
- `cross`
- `kick`
- `safe_shoot`
- `adjust`

注意：

- 当前行为树里没有 `decision == 'safe_shoot'` 的执行分支，因此这个值虽然能被生成，但不会被单独处理。

### 9.2 `GoalieDecide`

当前可能输出：

- `find`
- `retreat`
- `chase`
- `adjust`
- `kick`

### 9.3 `Kick`

逻辑：

- 启动时根据 `enableStableKick` 决定先稳一下还是直接冲球
- 踢球中会根据球是否移动、是否丢失、是否撞障碍物决定提前结束
- 踢球速度在 kickoff 场景下可被 `soft_kickoff` 限制

### 9.4 `RLVisionKick`

逻辑：

- 先减速
- 调 `VisualKick(start=true)`
- 期间根据球距离、是否丢球、控球成本、是否出界和超时退出
- 退出时切回 robocup walk

## 10. 定位逻辑

文件：

- `src/brain/src/locator.cpp`

当前定位器是粒子采样式匹配：

1. 用场地点标志构建理论地图
2. 随机生成初始粒子
3. 计算每个粒子的 marker 残差
4. 按概率重采样并逐步收缩
5. 收敛后输出最佳姿态

关键输入：

- 当前识别到的 `LCross / TCross / XCross / PenaltyPoint`
- `locator.min_marker_count`
- `locator.max_residual`

## 11. 控制接口

`RobotClient` 封装的常用动作包括：

- `moveHead`
- `setVelocity`
- `crabWalk`
- `moveToPoseOnField*`
- `standUp`
- `kickBall`
- `fancyKickBall`
- `RLVisionKick`
- `changeRobocupMode`
- `squatBlock`
- `squatUp`

其中最常用的是：

- `setVelocity`
- `moveHead`
- `RLVisionKick`
- `changeRobocupMode`

## 12. 当前版本已知限制

1. `safe_shoot` 决策没有行为树执行分支。
2. `goalie_mode == "guard"` 当前仍缺少明确的主动切入来源，现阶段主要通过 `decision == "intercept"` 走拦截动作。
3. 一些策略参数在 `config.yaml` 里存在，但并未真正接线到决策逻辑，详见 [参数与修改注意事项](./configuration.md)。
