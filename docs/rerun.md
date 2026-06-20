# Rerun 日志梳理

本文梳理当前代码里机器人向 `rerun` 发送的内容，重点回答三个问题：

1. 发了什么
2. 在哪个文件里发
3. 发送频率是什么

本文只统计当前代码路径下“实际会执行到”的发送逻辑；大量注释掉的旧日志、备用可视化不会计入“当前实际发送”。

## 总入口

- `brain` 侧统一发送入口是 [BrainLog::log](../src/brain/include/brain_log.h:37)，实际定义也在 [src/brain/include/brain_log.h](../src/brain/include/brain_log.h:37)。
- `BrainLog` 会根据配置决定是否发到 TCP 和/或文件，见 [src/brain/src/brain_log.cpp](../src/brain/src/brain_log.cpp:9)。
- `brain` 主循环频率目标是 `100Hz`，见 [src/brain/src/main.cpp](../src/brain/src/main.cpp:8) 和 [src/brain/src/main.cpp](../src/brain/src/main.cpp:32)。
- 只有在启用 `rerunLog.enable_tcp` 或 `rerunLog.enable_file` 时，彩色图像订阅才会建立，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:196)。

## 频率规则

- `主循环 100Hz`
  说明：由 `brain.tick()` 驱动。凡是 `Brain::tick()`、行为树 `tick()`、以及每 tick 都会调用的动作日志，理论上最高都接近 `100Hz`。
  位置：[src/brain/src/main.cpp](../src/brain/src/main.cpp:24)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:310)

- `按 ROS 回调频率`
  说明：检测、线段、odom、low_state、game controller、depth image 这些日志跟话题频率一致，不受 100Hz 主循环直接限制。

- `按 img_interval 降采样`
  说明：图像 `image/img` 不是每帧都发，而是每 `rerunLog.img_interval` 帧发一次。默认值是 `10`。
  位置：[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:120)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1686)

- `一次性/偶发`
  说明：静态场地图、切文件后重发、自定位成功/失败事件、恢复事件、视觉踢球退出原因等，都是条件触发。

## 当前实际发送

### 1. 框架与静态内容

| entity_path | 内容 | 触发位置 | 发送频率 |
| --- | --- | --- | --- |
| `performance/brain_tick` | 单次 `brain.tick()` 耗时标量 | [src/brain/src/main.cpp](../src/brain/src/main.cpp:32) | 每个主循环一次，约 `100Hz` |
| `field/mapLines` | 球场静态线、圆、门框 | [src/brain/src/brain_log.cpp](../src/brain/src/brain_log.cpp:55)、[src/brain/src/brain_log.cpp](../src/brain/src/brain_log.cpp:101) | 启动 `prepare()` 时一次；切新 `.rrd` 文件后再发一次 |

补充：

- `prepare()` 在初始化时调用，见 [src/brain/src/brain_log.cpp](../src/brain/src/brain_log.cpp:157)。
- 文件日志超过时长后会切文件并重发静态地图，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2944) 和 [src/brain/src/brain_log.cpp](../src/brain/src/brain_log.cpp:164)。

### 2. 主循环 100Hz 日志

这些日志都由 `Brain::tick()` 驱动，理论上最高约 `100Hz`。

| entity_path | 内容 | 触发位置 | 发送频率 |
| --- | --- | --- | --- |
| `debug/brain_tick` | 总体状态调试文本 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2652)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2655) | 每 tick 一次，约 `100Hz` |
| `debug/robot_state` | 机器人状态摘要，直接读取已有 `control_state`、`decision`、`ball_location_known`、`tm_ball_pos_reliable`、`wait_for_opponent_kickoff`、`gc_is_under_penalty`、`goalie_mode` 等状态并转发；典型值包括“等待启动 / 正在找球 / 已找到球 / 正在追球 / 已追到球，正在调整 / 已追到球，正在踢球”等 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:324)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3035)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3063) | 每 tick 一次，约 `100Hz`，且在 `tree->tick()` 之后发送，因此 `decision` 不会慢一拍 |
| `debug/robot_state_speech` | 状态播报候选文本；仅当状态变化时写入，用于核对某个中文状态会映射成哪句英文 TTS | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3086) | 仅在机器人状态变化时发送 |
| `debug/speak` | TTS 发送诊断，包含 `published / publisher not found / config not compatible / cooldown in process / repeat not allowed` 等原因，便于离线排查“为什么没出声” | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1108) | 每次调用 `speak()` 时按结果发送 |
| `debug/my_cost_scalar` | 本机 cost 标量 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2671) | 每 tick 一次 |
| `debug/my_lead_scalar` | 本机是否 lead 标量 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2672) | 每 tick 一次 |
| `image/detection_lag` | 检测延迟横条 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2984) | 每 tick 一次 |
| `performance/detection_lag_timeseries` | 检测延迟标量 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2992) | 每 tick 一次 |
| `image/fieldline_detection_lag` | 线段延迟横条 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3003) | 每 tick 一次 |
| `performance/fieldline_detection_lag_timeseries` | 线段延迟标量 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3011) | 每 tick 一次 |
| `image/gamecontrol_lag` | GameController 延迟横条 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3022) | 每 tick 一次 |
| `performance/gamecontrol_lag_timeseries` | GameController 延迟标量 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3030) | 每 tick 一次 |

### 3. 记忆态对象

这些对象虽然来自回调更新，但日志本身是由 `updateMemory()` 在主循环里刷新。

| entity_path | 内容 | 触发位置 | 发送频率 |
| --- | --- | --- | --- |
| `field/ball` | 本机记忆球位置，带 detected/known 状态 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:682)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:715) | 每 tick 一次，约 `100Hz` |
| `field/tmBall` | 队友共享球位置 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:722) | 每 tick 一次 |
| `field/robots` | 机器人记忆列表，逐个 `logRobot()` | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:730)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2515) | 每 tick 一次 |
| `field/mem_robots` | 空机器人列表时的清空路径 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2502) | 仅在机器人列表为空时发送 |
| `field/obstacles` | 障碍物记忆点、颜色、标签 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:654)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2559) | 每 tick 一次；depth 回调里还会再刷一次 |

注意：

- `field/mem_robots` 和 `field/robots` 路径不一致。现在“有数据时写 `field/robots`，为空时清 `field/mem_robots`”，这会导致 viewer 里看起来像清不掉旧数据。
- 相关代码位置在 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2497)。

### 4. 协作与队友状态

| entity_path | 内容 | 触发位置 | 发送频率 |
| --- | --- | --- | --- |
| `debug/handleCooperation` | 协作状态调试文本 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:401)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:404) | 每 tick 多次，约 `100Hz` |
| `field/teammate-<id>` | 队友场上位姿 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:440) | 每 tick，对每个有效队友一次 |
| `tm_ball-<id>` | 队友上报球位置 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:442) | 每 tick，对每个有效队友一次 |
| `debug/tm_cost_scalar_<id>` | 队友 cost 标量 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:466) | 每 tick，对每个 alive 队友一次 |
| `debug/tm_lead_scalar_<id>` | 队友 lead 标量 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:467) | 每 tick，对每个 alive 队友一次 |

### 5. GameController 相关

| entity_path | 内容 | 触发位置 | 发送频率 |
| --- | --- | --- | --- |
| `tick/gamecontrol` | 比赛状态文字覆盖框 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1280)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1408) | 每条 `/robocup/game_controller` 消息一次 |

### 6. 视觉检测回调

入口是 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1421) 的 `detectionsCallback()`，实际绘制在 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2385) 的 `logDetection()`。

| entity_path | 内容 | 触发位置 | 发送频率 |
| --- | --- | --- | --- |
| `image/detection_boxes` | 2D 检测框，带标签与颜色 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2477) | 每条 detection 消息一次 |
| `field/detection_points` | 检测目标在 field 坐标系中的点 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2482) | 每条 detection 消息一次 |
| `image/detection_boxes` `Clear::FLAT` | 无检测目标时清空检测框 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2387) | 仅在 `gameObjects.size()==0` 时 |
| `field/detection_points` `Clear::FLAT` | 无检测目标时清空点 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2388) | 仅在 `gameObjects.size()==0` 时 |
| `field/identified_markings` | 已识别的 `LCross/TCross/XCross/PenaltyPoint` | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2237) | 每条 detection 消息一次 |

补充：

- `Ball` 标签里会带球投影模式，如 `refined_plane` / `hold_last_valid` / `fallback_z0`，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2408) 之后的 `projectionModeToString()` 使用逻辑。

### 7. 场地线回调

入口是 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1485) 的 `fieldLineCallback()`。

| entity_path | 内容 | 触发位置 | 发送频率 |
| --- | --- | --- | --- |
| `field/det_lines` | 处理后的场地线，按类型着色并附标签 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1561) | 每条 line segment 消息一次 |
| `image/det_lines` | 图像平面上的线段 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1577) | 每条 line segment 消息一次 |

### 8. Odom 与本体状态回调

| entity_path | 内容 | 触发位置 | 发送频率 |
| --- | --- | --- | --- |
| `debug/odom_callback` | odom 文本调试 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1631)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1664) | 每条 `/odometer_state` 消息一次 |
| `field/robot` | 本机在 field 上的位姿、朝向、2m 圆 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1676) | 每条 `/odometer_state` 消息一次 |
| `debug/head_angles` | 头部 yaw/pitch 文本 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1679)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1683) | 每条 `/low_state` 消息一次 |

### 9. 彩色图像回调

入口是 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1686)。

| entity_path | 内容 | 触发位置 | 发送频率 |
| --- | --- | --- | --- |
| `debug/imageCallback` | 图像尺寸文本 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1696) | 每 `img_interval` 帧一次 |
| `image/img` | JPEG 压缩后的彩色图像 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1738) | 每 `img_interval` 帧一次 |

说明：

- 默认 `rerunLog.img_interval = 10`，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:120)。
- 所以实际频率是 `彩色图像话题频率 / img_interval`。

### 10. 深度回调

入口是 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2701)。

| entity_path | 内容 | 触发位置 | 发送频率 |
| --- | --- | --- | --- |
| `depth/depth_points` | 采样后的 3D 点云 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2628) | 每条 depth image 消息一次 |
| `depth/grid_mesh` | 占据网格 mesh | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2634) | 每条 depth image 消息一次 |
| `depth/ball_exclusion_box` | 球排除盒 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2644) | 每条 depth image 消息一次 |
| `field/obstacles` | 基于深度更新后的障碍物列表 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2701) 调用 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2534) | 每条 depth image 消息一次 |

### 11. RobotClient 动作层

这些日志不是固定话题回调，而是在行为树动作里调用 `RobotClient` 时触发。

| entity_path | 内容 | 触发位置 | 发送频率 |
| --- | --- | --- | --- |
| `debug/move_head` | 头部控制命令 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:56) | 每次 `moveHead()` |
| `RobotClient/squatDown` | 下蹲事件 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:122) | 每次 `squatBlock()` |
| `RobotClient/squatUp` | 起身事件 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:139) | 每次 `squatUp()` |
| `RobotClient/setVelocity_in` | 输入速度命令 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:193) | 每次 `setVelocity()` |
| `field/velocity` | 速度箭头 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:266) | 每次 `setVelocity()` |
| `RobotClient/setVelocity_out` | 限幅后的速度命令 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:278) | 每次 `setVelocity()` |
| `tick/time_to_collide` | 碰撞预计时间横条 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:370) | 仅在 `moveToPoseOnField(..., avoidObstacle=true)` 中触发 |
| `debug/avoidObstacle` | 避障决策文本 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:398) | 仅在避障介入时 |
| `debug/msecsToCollide` | 与障碍/机器人碰撞估计过程文本 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:723)、[src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:732) | 每次 `msecsToCollide()`，且对每个候选障碍/机器人可能多次 |

### 12. 行为树常驻可视化

这些通常在对应节点每次 `tick()` 时发，节点活跃时最高接近 `100Hz`。

| entity_path | 内容 | 触发位置 | 发送频率 |
| --- | --- | --- | --- |
| `debug/CamTrackBall` | 跟球调试文本 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:176)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:186) | `CamTrackBall` 活跃时每 tick |
| `image/track_ball` | 图像上的跟球容忍框 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:195) | `CamTrackBall` 活跃时每 tick |
| `debug/Assist` | Assist 节点调试文本 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:702)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:705) | `Assist` 活跃时每 tick |
| `field/kick_dir` | 踢球方向箭头 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:875)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:932) | `CalcKickDir` 活跃时每 tick |
| `debug/striker_decide` | 前锋决策调试文本 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:943)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:946) | `StrikerDecide` 活跃时每 tick |
| `tree/Decide` | 决策覆盖框 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1096)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1180) | `StrikerDecide` / `GoalieDecide` 活跃时每 tick |
| `tree/value_threat` | threat / kick value 覆盖框 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1107) | `StrikerDecide` 活跃时每 tick |
| `debug/Kick` | 踢球阶段调试文本 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1270)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1274) | `Kick` 活跃时每 tick |
| `debug/RLVisionKick` | 视觉踢球退出/切换原因 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1379)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1383)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1464) | 条件触发；节点活跃期间可能多次 |
| `recovery` | 跌倒恢复状态与事件 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:3371)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:3374)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:3385)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:3392)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:3401) | `CheckAndStandUp` 活跃时按条件发送 |

### 13. 自定位相关事件日志

这些不是固定频率流，而是自定位节点在成功/失败时写出的诊断。

| entity_path | 内容 | 触发位置 | 发送频率 |
| --- | --- | --- | --- |
| `SelfLocateLocal/SinglePenalty` | 单罚点定位成功/失败原因 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2004)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2020)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2036)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2050)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2057)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2066) | 节点触发且命中对应条件时 |
| `SelfLocateLocal/DoubleX` | 双 X 点定位成功/失败原因 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2082)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2095)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2111)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2125)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2132)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2141) | 节点触发且命中对应条件时 |

### 14. 标定调试日志

| entity_path | 内容 | 触发位置 | 发送频率 |
| --- | --- | --- | --- |
| `/field/vision_param` | 自动标定当前参数和最优参数文本 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:3541)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:3644) | `AutoCalibrateVision` 运行期间，按其内部状态机节奏发送 |

## 当前代码里有，但默认不发送

### 1. `field/visionBox`

- 函数存在于 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2370)。
- 但调用点被注释掉了，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1467)。
- 所以当前不会实际发送。

### 2. `field/hypos`

- 这是 `Locator` 自己的独立 rerun 流，发粒子点云，位置在 [src/brain/src/locator.cpp](../src/brain/src/locator.cpp:361) 和 [src/brain/src/locator.cpp](../src/brain/src/locator.cpp:370)。
- 但 `Locator::init()` 默认 `enableLog=false`，见 [src/brain/src/locator.cpp](../src/brain/src/locator.cpp:5)。
- `Brain::init()` 调用 `locator->init(...)` 时没有显式打开这个开关，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:173)。
- 因此当前默认不发。

### 3. 其它大量注释掉的备用流

当前代码里还有不少备用路径，但都被注释掉了，例如：

- `field/identified_goalposts`
- `field/det_lines_raw`
- `image/det_lines_raw`
- `robotframe/detection_points`
- `robotframe/det_lines`
- `robotframe/mem_robots`
- `field/projected_path`

这些不属于“当前实际发送”。

## 结论

当前最核心、最稳定会看到的 `rerun` 数据主要分为 6 类：

1. `100Hz` 主循环状态与性能
2. detection / field line / game controller / odom / low_state / depth 这些回调流
3. 低频降采样后的彩色图像 `image/img`
4. 行为树决策覆盖框与调试文本
5. 动作层 `RobotClient` 的控制日志
6. 若干按条件触发的自定位、恢复、标定诊断

如果后续要继续清理，我建议优先处理这两件事：

1. 统一 `field/robots` 和 `field/mem_robots` 路径
2. 给 `rerun` 日志再做一层分级，区分“默认常开”和“仅调试打开”
