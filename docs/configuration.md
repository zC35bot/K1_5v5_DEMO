# 参数与修改注意事项

本文档重点回答三个问题：

1. 某个参数到底从哪里进系统
2. 改完之后会影响哪段逻辑
3. 哪些参数现在其实没真正接线

## 1. 参数修改基本原则

### 1.1 先分清修改层级

这个仓库至少有三层参数来源：

- `brain/config/config.yaml`
- `vision/config/vision.yaml`
- launch 传参覆盖

另外很多部署脚本默认还会引用：

- `/opt/booster/vision.yaml`

所以改参数前，先确认当前运行的启动命令到底加载的是仓库内配置，还是系统外配置。

### 1.2 改参数时优先遵循的顺序

推荐顺序：

1. 先在仓库默认 YAML 上验证
2. 确认参数被代码真正声明并消费
3. 再考虑用 `*_local.yaml` 或 launch 覆盖

### 1.3 不要只看 YAML

这个仓库存在三类参数：

- A 类：已声明、已读取、已真正生效
- B 类：已声明或已读取，但当前逻辑中基本未实际驱动行为
- C 类：写在 YAML 里，但当前代码没有接线

后面会按这个思路整理。

## 2. Brain 参数总览

### 2.1 稳定生效的核心参数

这些参数我已经确认有明确代码落点。

#### `game.*`

- `game.team_id`
  - 用途：裁判机匹配、队友通信端口、主逻辑中的队伍标识
  - 风险：改错会导致收不到正确裁判机数据，也会导致队友通信跑到错误端口

- `game.player_id`
  - 用途：标识自己编号、队友状态数组下标、裁判机回包
  - 风险：编号冲突会导致协作逻辑混乱

- `game.field_type`
  - 用途：构建球场尺寸、理论线段、理论标志点
  - 风险：与真实场地不一致会直接导致定位偏差

- `game.player_role`
  - 用途：行为树初始角色
  - 风险：只影响初始角色，比赛中仍可能被 `RoleSwitchIfNeeded` 改写

- `game.player_start_pos`
  - 用途：初始站位
  - 风险：只接受 `left/right`

- `game.treat_person_as_robot`
  - 用途：把 `Person` 也当障碍机器人处理
  - 风险：正式比赛应保持 `false`

- `game.number_of_players`
  - 用途：队友协作、角色切换、在场人数判定

#### `robot.*`

- `robot.robot_height`
  - 用途：由投影估算目标俯仰角时使用
  - 风险：影响球、机器人、门柱的相对角度估算

- `robot.odom_factor`
  - 用途：缩放 odom 位移
  - 风险：改坏会导致整场定位漂移

- `robot.vx_factor`
  - 用途：`crabWalk` 里对前向速度做补偿
  - 风险：主要影响踢球时靠球轨迹

- `robot.yaw_offset`
  - 用途：`crabWalk` 时补偿方向误差
  - 风险：影响调向和踢球角度

- `robot.vx_limit / vy_limit / vtheta_limit`
  - 用途：所有速度指令统一限幅
  - 风险：影响全局移动能力，不建议局部调试时随便大改

#### `strategy.*`

- `strategy.ball_confidence_threshold`
  - 用途：低于阈值的球检测会被过滤
  - 代码落点：`detectProcessBalls()` 等

- `strategy.ball_confidence_decay_rate`
  - 当前状态：已接到 `BrainConfig`，但当前主逻辑里看不到实际消费

- `strategy.enable_stable_kick`
  - 用途：`Kick` 节点起脚前是否先稳定一下
  - 代码落点：`Kick::onStart`

- `strategy.ball_memory_timeout`
  - 用途：球多久没更新就认为位置未知
  - 代码落点：`updateBallMemory`

- `strategy.tm_ball_dist_threshold`
  - 用途：判断队友给的球位置是否可信
  - 代码落点：队友球可靠性判断逻辑

- `strategy.limit_near_ball_speed`
  - 用途：追球时靠近球会限速
  - 代码落点：`Chase::tick`

- `strategy.near_ball_speed_limit`
  - 用途：近球限速值

- `strategy.near_ball_range`
  - 用途：近球判定距离

- `strategy.soft_kickoff`
  - 用途：kickoff 阶段是否降速踢球
  - 代码落点：`Kick::onStart/onRunning`

- `strategy.soft_kickoff_speed`
  - 用途：kickoff 时的踢球速度

- `strategy.kick_range`
  - 用途：允许进入踢球分支的球距阈值
  - 代码落点：`StrikerDecide`、`Kick::onRunning`

- `strategy.kick_theta_range`
  - 用途：允许踢球时球偏角阈值
  - 代码落点：`StrikerDecide`

- `strategy.abort_kick_when_ball_moved`
  - 用途：踢球过程中如果球明显移动则中止
  - 代码落点：`Kick::onRunning`

- `strategy.abort_kick_ball_move_threshold`
  - 用途：球移动阈值

- `strategy.enable_auto_visual_kick`
  - 用途：允许 `StrikerDecide` 输出 `auto_visual_kick`
  - 代码落点：`StrikerDecide`

- `strategy.auto_visual_kick_enable_dist_min/max`
  - 用途：视觉踢球启用距离

- `strategy.auto_visual_kick_enable_angle`
  - 用途：视觉踢球启用偏角

- `strategy.enable_auto_visual_defend`
  - 用途：守门员自动视觉防守入口开关
  - 代码落点：`GoalieDecide`
  - 注意：当前相关分支逻辑很弱，基本属于预留状态

- `strategy.cooperation.enable_role_switch`
  - 用途：是否允许角色切换
  - 代码落点：`handleCooperation`

- `strategy.cooperation.ball_control_cost_threshold`
  - 用途：控球成本切换阈值
  - 代码落点：`handleCooperation`

#### `obstacle_avoidance.*`

- `depth_sample_step`
  - 用途：深度障碍物栅格采样步长
  - 风险：越小越准，但计算量更大

- `obstacle_min_height`
  - 用途：把深度点认定为障碍物的最小高度

- `grid_size`
  - 用途：障碍物栅格大小

- `max_x / max_y`
  - 用途：障碍物感知范围

- `exclusion_x / exclusion_y`
  - 用途：排除机器人自身身体区域

- `ball_exclusion_radius / ball_exclusion_height`
  - 用途：避免把球误判成障碍物

- `occupancy_threshold`
  - 用途：栅格占用判定阈值

- `collision_threshold`
  - 用途：计算与障碍物碰撞的横向距离阈值

- `safe_distance`
  - 用途：通用避障安全距离

- `avoid_secs`
  - 用途：避障动作持续时间

- `enable_freekick_avoid`
  - 用途：任意球摆位时是否把球当障碍物

- `freekick_start_placing_safe_distance`
  - 用途：任意球摆位时专用安全距离

- `freekick_start_placing_avoid_secs`
  - 用途：任意球摆位时专用避障时长

- `obstacle_memory_msecs`
  - 用途：障碍物记忆保留时间

- `avoid_during_chase`
  - 用途：追球时是否启用避障
  - 代码落点：`Chase::tick`

- `chase_ao_safe_dist`
  - 用途：追球避障安全距离

- `avoid_during_kick`
  - 用途：踢球过程中是否因前方障碍而中止/后退
  - 代码落点：`Kick`

- `kick_ao_safe_dist`
  - 用途：踢球避障距离

- `kick_ao_use_shoot`
  - 当前状态：已读取，但当前版本没有看到稳定有效的实际分支用途

- `always_turn_left`
  - 用途：避障转向强制左转
  - 代码落点：`RobotClient::moveToPoseOnField2`

#### `locator.*`

- `locator.min_marker_count`
  - 用途：最少要看到多少个 marker 才开始定位

- `locator.max_residual`
  - 用途：定位残差容忍度

#### 其他

- `enable_com`
  - 用途：打开/关闭队友通信

- `rerunLog.enable_tcp / server_ip / enable_file / log_dir / max_log_file_mins / img_interval`
  - 用途：调试日志输出

- `vision.image_topic / depth_image_topic`
  - 用途：Brain 订阅图像与深度图的话题

- `vision.cam_pixel_width / cam_pixel_height / cam_fov_x / cam_fov_y`
  - 用途：视觉跟球、视角估计

- `game_control_ip`
  - 用途：向裁判机发送存活回包的目标 IP

- `recovery.retry_max_count`
  - 用途：自动起身最大重试次数

## 3. Brain 中“存在但当前未完全可靠”的参数

这些参数不是不能改，而是当前版本别把它们当成“改了必然有清晰效果”的正式开关。

### 3.1 已读取，但当前主逻辑没有真正落地成明确行为

- `strategy.enable_bypass`
- `strategy.enable_shoot`
- `strategy.enable_directional_kick`
- `strategy.power_shoot.enable`
- `strategy.power_shoot.use_for_kickoff`
- `strategy.shoot.xmin/xmax/ymin/ymax`
- `strategy.power_shoot.xmin/xmax/ymin/ymax`

原因：

- 这些参数在 `StrikerDecide` 中被读取，甚至局部变量也算出来了
- 但当前决策落地仍主要是 `find / assist / chase / auto_visual_kick / adjust / kick / cross`
- 比如 `safe_shoot` 虽然会被生成，但行为树没有对应执行分支

结论：

- 这些参数更像“开发中遗留/预留逻辑”
- 如果要真正启用，需要先补行为树分支和动作节点逻辑

### 3.2 已接入配置，但当前没有看到有效消费

- `strategy.ball_confidence_decay_rate`

这个值已经进了 `BrainConfig`，但当前主逻辑里看不到它参与球记忆衰减判断。

## 4. Brain 中“写在 YAML 里，但当前未接线”的参数

以下字段出现在 `src/brain/config/config.yaml` 中，但当前代码没有确认到实际消费：

- `strategy.abort_shoot_when_ball_moved`
- `strategy.limit_speed_near_border`
- `strategy.near_border_speed_limit`
- `strategy.near_border_dist_threshold`
- `debug.log_ball_pos`
- `rerunLog.enable`

说明：

- `ball_predictor.*` 已经接到 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1031) 的 `updateBallPrediction()`，不再属于“YAML 写了但完全未接线”
- `rerunLog.enable` 在代码里甚至只剩一行注释
- 这些字段不要只改 YAML 就期待结果变化

## 5. Vision 参数总览

### 5.1 相机与标定

- `camera.type`
  - 用途：选择相机类型字符串
  - 注意：当前不同类型大多还是映射到同一组话题

- `camera.intrin.fx / fy / cx / cy`
  - 用途：投影、回投影、位姿估计
  - 风险：错了会导致所有三维位置都有系统性偏差

- `camera.extrin`
  - 用途：相机到头部的外参
  - 风险：这是最关键的几何参数之一

- `camera.pitch_compensation / yaw_compensation / z_compensation`
  - 用途：在原始外参之上做在线补偿
  - 来源：可静态配置，也可通过 `/booster_vision/cal_param` 动态更新

### 5.2 检测模型

- `detection_model.model_path`
  - 用途：检测模型路径

- `detection_model.confidence_threshold`
  - 用途：检测器整体阈值

- `detection_model.nms_threshold`
  - 用途：NMS 阈值

- `detection_model.classnames`
  - 用途：类别名称表
  - 风险：必须与模型输出类别顺序一致

- `detection_model.post_process.single_ball_assumption`
  - 用途：多个球时只保留最高置信度球
  - 代码落点：`ProcessData`

- `detection_model.post_process.confidence_thresholds`
  - 用途：分类别阈值
  - 代码落点：`ProcessData`

### 5.3 分割模型

- `segmentation_model.model_path`
- `segmentation_model.confidence_threshold`
- `segmentation_model.nms_threshold`

### 5.4 深度与位姿估计

- `use_depth`
  - 用途：是否启用深度同步

- `ball_pose_estimator.*`
  - 用途：球位姿估计器参数

- `human_like_pose_estimator.*`
  - 用途：人形/门柱等估计器参数

- `field_marker_pose_estimator.refine`
  - 当前主流程里未看到明显消费

- `field_marker_pose_estimator.line_segment_area_threshold`
  - 用途：拟合场线时的轮廓面积阈值

### 5.5 标定参数

- `calibration.sync_max_time_diff_ms`
  - 用途：标定时图像与头部位姿同步容忍时间

- `calibration.offset.exclude_distance`
  - 用途：偏移标定中剔除离群点阈值

- `calibration.offset.zero_translation`
  - 用途：是否固定平移，只优化角度

- `calibration.offset.field_marker_path`
  - 用途：偏移标定使用的理论场地点位文件

- `calibration.handeye.calibration_time`
- `calibration.handeye.reprojection_error`
  - 用途：记录上次标定结果，不直接驱动运行逻辑

### 5.6 杂项

- `misc.save_data_nonstationary`
  - 用途：控制 DataLogger 是否只保存非静止数据

## 6. 修改参数时的实操建议

### 6.1 想调“追球靠不靠得住”

优先看：

- `strategy.limit_near_ball_speed`
- `strategy.near_ball_speed_limit`
- `strategy.near_ball_range`
- `robot.vx_limit / vy_limit / vtheta_limit`
- `robot.vx_factor / yaw_offset`

### 6.2 想调“踢球前是否更稳”

优先看：

- `strategy.enable_stable_kick`
- `strategy.kick_range`
- `strategy.kick_theta_range`
- `strategy.abort_kick_when_ball_moved`
- `strategy.abort_kick_ball_move_threshold`

### 6.3 想调“避障太激进/太保守”

优先看：

- `obstacle_avoidance.safe_distance`
- `obstacle_avoidance.collision_threshold`
- `obstacle_avoidance.grid_size`
- `obstacle_avoidance.occupancy_threshold`
- `obstacle_avoidance.avoid_during_chase`
- `obstacle_avoidance.chase_ao_safe_dist`

### 6.4 想调“视觉球距不准 / 场线不准”

优先看：

- `camera.intrin.*`
- `camera.extrin`
- `camera.pitch_compensation / yaw_compensation / z_compensation`
- `ball_pose_estimator.*`
- `field_marker_pose_estimator.line_segment_area_threshold`

### 6.5 想调“队友协作总抢球 / 不抢球”

优先看：

- `enable_com`
- `game.number_of_players`
- `strategy.cooperation.enable_role_switch`
- `strategy.cooperation.ball_control_cost_threshold`

## 7. 当前仓库建议优先修复的问题

从可维护性角度，建议优先处理这些问题：

1. 给 `safe_shoot` 补行为树执行分支，或者不要再生成这个 decision。
2. 清理 `config.yaml` 中未接线参数，或者把它们真正接上。
3. 修正 `assist.sh` / `calibrate.sh` 指向不存在行为树的问题。
4. 明确 `vision` 实际运行时是否以仓库内 `vision.yaml` 还是 `/opt/booster/vision.yaml` 为准。
