# Rerun 日志梳理

本文按“发了什么、在哪个文件、什么频率”整理当前机器人向 `rerun` 的主要输出。这里只统计当前代码路径里实际会走到的发送逻辑。

## 总入口

- 统一发送入口是 [src/brain/include/brain_log.h](../src/brain/include/brain_log.h:37) 的 `BrainLog::log(...)`。
- `BrainLog` 根据 `rerunLog.enable_tcp / rerunLog.enable_file` 决定发到 TCP 还是本地文件，见 [src/brain/src/brain_log.cpp](../src/brain/src/brain_log.cpp:7)、[src/brain/src/brain_log.cpp](../src/brain/src/brain_log.cpp:9)、[src/brain/src/brain_log.cpp](../src/brain/src/brain_log.cpp:20)。
- 静态场地图在 `prepare()` 和切分新 `.rrd` 文件后重发，见 [src/brain/src/brain_log.cpp](../src/brain/src/brain_log.cpp:100)、[src/brain/src/brain_log.cpp](../src/brain/src/brain_log.cpp:157)、[src/brain/src/brain_log.cpp](../src/brain/src/brain_log.cpp:164)。
- 主循环目标频率是 `100Hz`，并在每拍写 `performance/brain_tick`，见 [src/brain/src/main.cpp](../src/brain/src/main.cpp:8)、[src/brain/src/main.cpp](../src/brain/src/main.cpp:32)。

## 频率规则

- `主循环 100Hz`
  由 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:320) 的 `Brain::tick()` 驱动，凡是每拍固定执行的日志理论上都接近 `100Hz`。
- `按 ROS 回调频率`
  detection、field line、game controller、odometer、low state、depth image 这类日志跟各自话题频率走。
- `按 img_interval 降采样`
  彩图 `image/img` 不是每帧都发，而是按 `rerunLog.img_interval` 抽样，见 [src/brain/config/config.yaml](../src/brain/config/config.yaml:108)、[src/brain/config/config.yaml](../src/brain/config/config.yaml:114)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2259)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2311)。
- `一次性/条件触发`
  静态场地图、自定位结果、恢复事件、视觉踢球退出原因等属于条件触发。

## 主循环类日志

| entity_path | 内容 | 文件位置 | 发送频率 |
| --- | --- | --- | --- |
| `performance/brain_tick` | 单次 `brain.tick()` 耗时 | [src/brain/src/main.cpp](../src/brain/src/main.cpp:32) | 每个主循环一次，约 `100Hz` |
| `debug/brain_tick` | 当前关键状态文本摘要 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3236) | 每 tick 一次 |
| `debug/robot_state` | 机器人高层状态摘要 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3616)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3629) | 每 tick 一次，且在 `tree->tick()` 之后 |
| `debug/robot_state_speech` | 状态到英文播报词的映射结果 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3637)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3639) | 仅在状态变化时 |
| `debug/speak` | TTS 发送诊断 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1602)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1607) | 每次 `speak()` 调用按结果写入 |
| `debug/my_cost_scalar` | 本机 cost 标量 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3252) | 每 tick 一次 |
| `debug/my_lead_scalar` | 本机 lead 标量 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3253) | 每 tick 一次 |
| `image/detection_lag` | 检测延迟横条 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3565) | 每 tick 一次 |
| `performance/detection_lag_timeseries` | 检测延迟时间序列 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3573) | 每 tick 一次 |
| `image/fieldline_detection_lag` | 线段延迟横条 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3584) | 每 tick 一次 |
| `performance/fieldline_detection_lag_timeseries` | 线段延迟时间序列 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3592) | 每 tick 一次 |
| `image/gamecontrol_lag` | GameController 延迟横条 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3603) | 每 tick 一次 |
| `performance/gamecontrol_lag_timeseries` | GameController 延迟时间序列 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3611) | 每 tick 一次 |

## 记忆与场上对象

| entity_path | 内容 | 文件位置 | 发送频率 |
| --- | --- | --- | --- |
| `field/ball` | 本机记忆球位置 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:982)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1015) | 每 tick 一次 |
| `field/tmBall` | 队友共享球位置 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1022) | 每 tick 一次 |
| `field/robots` | 机器人记忆列表 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3096) | 每 tick 一次 |
| `field/mem_robots` | 机器人列表为空时的清空路径 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3083) | 仅在列表为空时 |
| `field/obstacles` | 障碍物记忆列表 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3115)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3140) | 每 tick 一次；深度回调里也会刷 |

注意：

- 目前“有数据时写 `field/robots`，为空时清 `field/mem_robots`”这一点还不统一，viewer 里可能看起来像旧机器人没清掉，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3083)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3096)。

## 球路预测与守门拦截

| entity_path | 内容 | 文件位置 | 发送频率 |
| --- | --- | --- | --- |
| `field/ball_prediction` | 球路预测点列 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1288)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1374) | 每 tick 一次；无效时清空 |
| `field/ball_breach_point` | 预测穿过守门拦截线的点 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1452)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1466) | `ballWillBreach=true` 时写，否则清空 |
| `field/ball_intercept_point` | 建议拦截点 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1459)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1467) | `ballWillBreach=true` 时写，否则清空 |
| `performance/ball_will_breach` | 是否会穿越拦截线 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1307)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1448) | 每 tick 一次 |
| `debug/ball_prediction` | 预测摘要、跳过原因、拦截信息 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1329)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1359)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1472) | 每 tick 一次或按条件说明 |

## 协作与队友状态

| entity_path | 内容 | 文件位置 | 发送频率 |
| --- | --- | --- | --- |
| `debug/handleCooperation` | 协作与角色仲裁调试文本 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:412)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:415) | 每 tick 多次 |
| `field/teammate-<id>` | 队友位姿与标签 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:533)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:544) | 每 tick，对每个有效队友一次 |
| `tm_ball-<id>` | 队友上报球位置 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:545) | 每 tick，对每个有效队友一次 |
| `debug/tm_cost_scalar_<id>` | 队友 cost 标量 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:570) | 每 tick，对每个 alive 队友一次 |
| `debug/tm_lead_scalar_<id>` | 队友 lead 标量 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:571) | 每 tick，对每个 alive 队友一次 |

补充：

- 当前 `field/teammate-<id>` 标签里除了 `ID / Cost / State`，还会显示 `Role / Down / Cap: [decisionId] S=striker P=supporter`，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:533)。
- 队长仲裁主逻辑在 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:641) 之后，包含旧 `cmd` 兼容、前场集合、远端守门员采纳、本地回退、战术角色写回和总结日志。

## GameController、视觉、里程计、深度

| entity_path | 内容 | 文件位置 | 发送频率 |
| --- | --- | --- | --- |
| `tick/gamecontrol` | 比赛状态覆盖框 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1853)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1981) | 每条 game controller 消息一次 |
| `image/detection_boxes` | 2D 检测框 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2966)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3058) | 每条 detection 消息一次 |
| `field/detection_points` | 检测点投到 field 的位置 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2969)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3063) | 每条 detection 消息一次 |
| `field/identified_markings` | 已识别场地标记 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2818) | 每条 detection 消息一次 |
| `field/det_lines` | 处理后的场地线 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2058)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2134) | 每条 line segment 消息一次 |
| `image/det_lines` | 图像上的场地线 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2150) | 每条 line segment 消息一次 |
| `debug/odom_callback` | odom 文本 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2204)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2237) | 每条 odometer 消息一次 |
| `field/robot` | 本机位姿与朝向 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2249) | 每条 odometer 消息一次 |
| `debug/head_angles` | 头部 yaw/pitch | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2252)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2256) | 每条 low state 消息一次 |
| `image/img` | JPEG 压缩彩图 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2259)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2311) | 每 `img_interval` 帧一次 |
| `depth/depth_points` | 采样后的 3D 点云 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3209) | 每条 depth image 消息一次 |
| `depth/grid_mesh` | 深度占据网格 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3215) | 每条 depth image 消息一次 |
| `depth/ball_exclusion_box` | 球排除盒 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3225) | 每条 depth image 消息一次 |

补充：

- 球检测框标签里会附带投影模式 `refined_plane / hold_last_valid / fallback_z0`，对应逻辑见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2999)、[src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3018)。

## RobotClient 动作层

| entity_path | 内容 | 文件位置 | 发送频率 |
| --- | --- | --- | --- |
| `debug/move_head` | 头部控制命令 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:56) | 每次 `moveHead()` |
| `RobotClient/squatDown` | 下蹲事件 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:122) | 每次 `squatBlock()` |
| `RobotClient/squatUp` | 起身事件 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:139) | 每次 `squatUp()` |
| `RobotClient/setVelocity_in` | 输入速度命令 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:193) | 每次 `setVelocity()` |
| `field/velocity` | 速度箭头 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:266) | 每次 `setVelocity()` |
| `RobotClient/setVelocity_out` | 限幅后的速度命令 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:278) | 每次 `setVelocity()` |
| `tick/time_to_collide` | 碰撞预计时间横条 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:370) | 仅在避障求解时 |
| `debug/avoidObstacle` | 避障决策文本 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:398) | 仅在避障介入时 |
| `debug/msecsToCollide` | 碰撞估计过程文本 | [src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:723)、[src/brain/src/robot_client.cpp](../src/brain/src/robot_client.cpp:732) | 每次 `msecsToCollide()` 可能多次 |

## 行为树常驻可视化

| entity_path | 内容 | 文件位置 | 发送频率 |
| --- | --- | --- | --- |
| `debug/CamTrackBall` | 跟球调试文本 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:190) | 节点活跃时每 tick |
| `image/track_ball` | 图像上的跟球辅助框 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:199) | 节点活跃时每 tick |
| `debug/Assist` | Assist 调试文本 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:946) | 节点活跃时每 tick |
| `field/kick_dir` | 踢球方向箭头 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1243) | 节点活跃时每 tick |
| `debug/striker_decide` | 前锋决策调试文本 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1257) | 节点活跃时每 tick |
| `tree/Decide` | 决策覆盖框 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1735)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1835) | 节点活跃时每 tick |
| `tree/value_threat` | threat / kick value 覆盖框 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1750) | 节点活跃时每 tick |
| `debug/Kick` | 踢球阶段调试文本 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:1929) | 节点活跃时每 tick |
| `debug/RLVisionKick` | 视觉踢球退出/切换原因 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2038)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2119) | 条件触发 |
| `debug/intercept` | 守门员拦截开始日志 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2180) | `Intercept` 启动时一次 |
| `recovery` | 跌倒恢复状态与事件 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:4029)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:4032)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:4043)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:4050)、[src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:4059) | 按条件触发 |

## 当前有代码但默认不发

- `field/visionBox`
  函数在 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:2951)，但当前没有正常发送链路接入。
- `field/hypos`
  这是 `Locator` 自己的 rerun 流，默认没打开；若后续要用，需要再检查 `Locator::init()` 的日志开关。

## 结论

当前最核心、最稳定会看到的 `rerun` 数据主要分成六类：

1. `100Hz` 主循环状态与性能
2. 球、机器人、障碍物等记忆态对象
3. 球路预测与守门拦截辅助可视化
4. 队友状态、队长仲裁与通信调试
5. detection / line / game controller / odometer / low state / depth 这些回调流
6. 行为树与 `RobotClient` 的执行侧调试日志
