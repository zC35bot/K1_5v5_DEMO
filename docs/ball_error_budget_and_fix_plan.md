# Ball Error Budget And Fix Plan

## 标签归类

为了避免后续把不同层面的改动混在一起，这里先固定四个标签：

- `视觉识别增强`：改检测器、分类器、bbox 质量，目标是“先把球看对”。
- `视觉投影/测距增强`：改 `vision` 里的投影、地面拟合、深度几何，目标是“把球量准”。
- `定位增强`：改 `Locator`、`robotPoseToField`、marker/场线修正，目标是“把机器人自己放准到场地里”。
- `观测稳定化 / 状态估计增强`：不直接改检测器和定位器，而是改记忆、置信度衰减、时序平滑、跨帧保持，目标是“不要把一帧异常放大成决策抖动”。

本轮准备先落地的 `ball_confidence_decay_rate -> updateBallMemory()`，明确属于：

- `观测稳定化 / 状态估计增强`

它不是：

- `球路预测`
- `视觉识别增强`
- `Locator 定位增强`

它做的事情是：在 `brain` 内把“最后一次看到的球”从一个硬 3 秒保留的布尔状态，改成“随时间持续降权的观测状态”。这会先影响 `ball_location_known`、球记忆可靠性和队友共享时的置信度语义；不会直接改变 `vision` 端的 bbox，也不会直接改变 `Locator` 的场地定位结果。

同样地，`EstimateProjection()` 在局部平面拟合失败时“短时保留上一帧有效 refined projection”的改动，明确属于：

- `视觉投影/测距增强`

它也不是：

- `球路预测`

它做的事情是：当局部深度地面拟合只是单帧掉点、单帧 RANSAC 失败时，不要立刻从 refined plane 阶跃回退到 `z = 0`。而是先保留 1 到 2 帧上一帧的有效 refined projection，只有连续失败或当前差异过大时才真正回退。

## 目标

本文档整理当前球定位链路中的主要误差来源，解释为什么误差会累积，以及在短周期内应该优先改哪些点。

这里重点回答三个问题：

1. 误差到底出在哪些环节
2. 为什么这些误差不是独立的，而是会层层叠加
3. 如果只能优先做 1 到 2 个改动，最值得先做什么

## 先说结论

基于当前仓库实现，如果目标是在最小改动下优先缓解“球测远、球坐标跳、追球头角不稳”这三类问题，我会优先做：

1. 让 `ball_confidence_decay_rate` 真正接进 `updateBallMemory()`
2. 让 `EstimateProjection()` 在局部平面拟合失败时，不要立刻单帧硬回退 `z=0`，而是优先保留上一帧有效投影 1 到 2 帧

原因很直接：

- 这两项都不需要重做系统结构
- 一项改 `brain` 记忆层，一项改 `vision` 投影层
- 都可以直接降低“误差被短时异常放大”的程度
- 都比“重做全套视觉几何”更适合短周期冲刺

## 当前球定位主链路

当前主链路不是“视觉直接给最终球场坐标”，而是两段式：

1. `vision` 先输出 `position_projection`
2. `brain` 再把机器人系下的球位置，变换到 field 系

关键路径：

- [src/vision/src/vision_node.cpp](../src/vision/src/vision_node.cpp) line 406
- [src/vision/src/pose_estimator/pose_estimator.cpp](../src/vision/src/pose_estimator/pose_estimator.cpp) line 197
- [src/brain/src/brain.cpp](../src/brain/src/brain.cpp) line 2064
- [src/brain/src/brain.cpp](../src/brain/src/brain.cpp) line 2073

这意味着：

- 视觉误差先形成 `球_机体系` 误差
- 然后再叠加机器人定位误差，形成 `球_field系` 误差

## 误差累积链路

可以把误差分成四层。

### 第一层：检测像素层

这里的误差源包括：

- bbox 本身抖动
- bbox 底中点不等于球真实接地点
- 球是球面，bbox 底部像素不等于接地点

这些误差的共同特点是：

- 在图像里只差 1 到 2 个像素
- 但一旦投影到 1.5m 到 3m 的球上，会直接变成厘米级误差

对应代码位置：

- [src/vision/src/pose_estimator/pose_estimator.cpp](../src/vision/src/pose_estimator/pose_estimator.cpp) line 191
- [src/vision/src/pose_estimator/pose_estimator.cpp](../src/vision/src/pose_estimator/pose_estimator.cpp) line 229

### 第二层：相机与投影几何层

这里的误差源包括：

- `p_eye2base` 三段链路误差
- `/head_pose` 与图像不同步
- 固定 `z=0` 投影本身对俯仰/外参敏感
- 新增地面拟合在远场时点少、噪声大

当前相机位姿链路是：

```text
p_eye2base = p_head2base * p_headprime2head * p_eye2head
```

其中任何一段有偏差，都会直接影响投影交点。

对应代码位置：

- [src/vision/src/vision_node.cpp](../src/vision/src/vision_node.cpp) line 111
- [src/vision/src/pose_estimator/pose_estimator.cpp](../src/vision/src/pose_estimator/pose_estimator.cpp) line 98
- [src/vision/src/pose_estimator/pose_estimator.cpp](../src/vision/src/pose_estimator/pose_estimator.cpp) line 197

### 第三层：`brain` 机体系到场地系转换层

视觉给出的 `position_projection` 并不是最终比赛决策使用的坐标。

`brain` 还会做：

```text
球_field系位置 = transCoord(球_机体系, robotPoseToField)
```

所以只要：

- `球_机体系` 本身有误差
- `robotPoseToField` 有误差

最终 `球_field系` 一定是两者叠加。

对应代码位置：

- [src/brain/src/brain.cpp](../src/brain/src/brain.cpp) line 2064
- [src/brain/src/brain.cpp](../src/brain/src/brain.cpp) line 2073

### 第四层：记忆与多机共享层

一旦球位置进入：

- `ball memory`
- `tmBall`
- 队友通信广播

它就不再只是“单帧误差”，而会变成“时序误差”和“多机误差”。

典型问题：

- 球已经跳了，但旧位置还保留 3 秒
- 两台机器人都各自漂过，再把各自球坐标互相广播
- 最后协作层拿到的是“被双重定位误差污染过的球坐标”

对应代码位置：

- [src/brain/src/brain.cpp](../src/brain/src/brain.cpp) line 478
- [src/brain/src/brain.cpp](../src/brain/src/brain.cpp) line 499
- [src/brain/src/brain.cpp](../src/brain/src/brain.cpp) line 682
- [src/brain/src/brain_communication.cpp](../src/brain/src/brain_communication.cpp) line 361

## 为什么说“不可控”

“不可控”不是说系统完全不能调，而是说当前链路里有三处非常硬的放大器。

### 1. 地面拟合是局部、小样本、远场退化的

当前局部平面拟合只看球周围 ROI。

这在近场通常没问题，但在远场会同时遇到：

- 球像素很小
- ROI 虽然扩展了，但有效深度点还是不多
- 深度噪声在远处更大
- RANSAC 质量门控更容易失败

结果就是：

- 近场可能走新算法
- 远场很容易掉回旧算法

这意味着算法行为本身不是连续的。

### 2. fallback 是阶跃，不是渐变

当前逻辑是：

- 平面可用：用拟合平面
- 平面不可用：立即退回 `z=0`

中间没有缓冲，没有上一帧保持，没有状态连续性约束。

所以两帧之间只要有一次：

- 点不够
- 置信度不够
- 法向不够像地面

结果就会从“新算法解”瞬间跳到“旧算法解”。

这就是球位置在 2 到 3 米处会突然抖一下、跳一下的原因之一。

### 3. 定位是离散修正，不是持续稳定闭环

当前 `Locator` 的更新依赖能看到 marker。

低头找球时，常见情况是：

- 很长时间看不到 marker
- `robotPoseToField` 主要靠 odom 漂
- 抬头一帧看到 marker 后，定位突然修一大步

只要 `球_机体系` 位置还是一样的，`transCoord` 一变，`球_field系` 也会跟着跳。

这不是视觉投影本身的问题，而是“视觉投影误差 + 场地定位跳变”叠加后的结果。

## 我会怎么改

下面不是“长期最优解”，而是“短周期、低风险、收益高”的改法。

### 优先改 1：把 `ball_confidence_decay_rate` 真正接进 `updateBallMemory()`

当前状态：

- `strategy.ball_confidence_decay_rate` 已经声明了
- 也已经读进 `BrainConfig`
- 但当前主逻辑里没有真正消费它

对应代码位置：

- [src/brain/src/brain.cpp](../src/brain/src/brain.cpp) line 45
- [src/brain/src/brain.cpp](../src/brain/src/brain.cpp) line 227
- [src/brain/src/brain.cpp](../src/brain/src/brain.cpp) line 682
- [src/brain/config/config.yaml](../src/brain/config/config.yaml)

我会怎么做：

1. 在 `updateBallMemory()` 中引入基于时间的球置信度衰减
2. 不再只在超时点把球从“已知”直接打成“未知”
3. 而是在超时前就逐步降低球的可信度
4. 当置信度掉到阈值以下时，再认为球不可可靠使用

这样做的收益：

- 球短时抖动或临时丢失时，不会立刻把旧位置当成完全可信
- 后续的 `kickDir`、`tmBall`、决策状态机会更早感知“这个球不可靠了”
- 对多机共享也更安全，因为低置信球可以少参与仲裁

为什么这项优先级高：

- 只改 `brain`
- 不碰视觉和定位主链路
- 风险低
- 能直接减轻“错误球位置在记忆里硬保留 3 秒”的问题

### 优先改 2：地面拟合失败时，先保留上一帧有效投影

当前状态：

- `EstimateProjection()` 一旦判定平面不可用，就直接退回旧投影

对应代码位置：

- [src/vision/src/pose_estimator/pose_estimator.cpp](../src/vision/src/pose_estimator/pose_estimator.cpp) line 197

我会怎么做：

1. 在 `BallPoseEstimator` 中缓存上一帧有效的投影结果
2. 如果当前帧：
   - depth 空
   - 点数不够
   - 平面质量不过
3. 不要立刻回 `z=0`
4. 优先保留上一帧结果 1 到 2 帧
5. 连续数帧都失败后，再回退到旧算法

这个改动本质上是给 `EstimateProjection()` 加一个很轻量的时间连续性约束。

这样做的收益：

- 抑制单帧深度抖动引起的投影阶跃
- 球在静止或缓慢移动时，位置明显更稳
- 不需要重写整套平面拟合

为什么这项优先级高：

- 改动集中
- 算法意图简单
- 对“新旧解来回跳”是直接打点修复

## 为什么暂时不先改别的

### 不先改“近距离直接优先用深度球心”

这个方向长期是对的，尤其近距离球：

- 深度球面拟合理论上能少掉一部分接地点投影误差

但短期我不会把它排在最前面，因为：

- `brain` 当前消费的是 `position_projection`
- 把 `position` 提升为主来源，会牵涉更多接口与策略侧假设
- 一旦近距离深度点质量不稳，反而会引入新的切换逻辑

我会把它放在 `#1 + #3` 做完之后。

### 不先改“Locator 大重构”

定位跳变确实是大问题，但它牵涉面太大：

- marker 定位
- 场线辅助
- odom 融合
- 行为树低头/抬头扫描策略

15 天冲刺期，我会先把球链路里“最容易跳的那一层”压住，而不是先动最深的定位系统。

### 不先改“多机 ball 共享 EMA”

这个改动是值得做的，但它属于第二层收益：

- 只有当本机球定位和本机球记忆先稳定下来后
- 多机平滑才不会只是“平滑错误”

所以它适合放在 `#1` 完成之后。

## 推荐的改动顺序

如果下一步真要开始动代码，我建议：

1. 先改 `updateBallMemory()`，把 `ball_confidence_decay_rate` 接起来。
2. 再改 `EstimateProjection()`，加上一帧结果保留，不再单帧硬回退。
3. 再考虑近距离球使用深度结果主导。
4. 再考虑 `tmBall` 的 EMA 或滞后。
5. 最后才考虑更大规模的 `Locator` 修正策略。

## 本轮新增改动

这轮除了前面已经接入的 `ball_confidence_decay_rate` 和 `hold_last_valid` 之外，又补了三件事：

1. `CamTrackBall` 收紧居中容忍框
2. 把球投影模式暴露到 rerun
3. 增加 `Ball` 与 `PenaltyPoint` 的近邻冲突抑制

### 1. `CamTrackBall` 收紧容忍框

位置：

- [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp)

改动：

- `pixToleranceX/Y` 从原来的约 `30%` 视野宽高收紧到 `12%`

目的：

- 原先头部会在球仍明显偏离图像中心时提前判定“已居中”
- 结果就是 chase 还能用，因为球总体还在视野里，但单纯盯球会出现“看得到却盯不住”

这项改动属于：

- `观测稳定化 / 状态估计增强`

它不是改球测距本身，而是改球被看到后头部是否持续把它拉回中心。

### 2. rerun 直接显示球投影来源

位置：

- [src/vision/include/booster_vision/pose_estimator/pose_estimator.h](../src/vision/include/booster_vision/pose_estimator/pose_estimator.h)
- [src/vision/src/pose_estimator/pose_estimator.cpp](../src/vision/src/pose_estimator/pose_estimator.cpp)
- [src/vision/src/vision_node.cpp](../src/vision/src/vision_node.cpp)
- [src/brain/include/types.h](../src/brain/include/types.h)
- [src/brain/src/brain.cpp](../src/brain/src/brain.cpp)

做法：

- `BallPoseEstimator` 现在会记住当前帧球投影到底来自：
  - `refined_plane`
  - `hold_last_valid`
  - `fallback_z0`
- `vision_node` 把这个模式编码进现有 `DetectedObject.position_confidence`
- `brain` 收到后把模式名直接拼到 rerun 的球标签里

你在 rerun 里会直接看到：

- `Ball[refined_plane]`
- `Ball[hold_last_valid]`
- `Ball[fallback_z0]`

这项改动属于：

- `视觉投影/测距增强`
- 更准确地说，是这一层的“可观测性增强”

它的目标不是直接减少误差，而是先把静态抖动的来源按帧拆清楚，方便继续调参。

### 3. `Ball` 与 `PenaltyPoint` 冲突抑制

位置：

- [src/vision/src/vision_node.cpp](../src/vision/src/vision_node.cpp)

做法：

- 如果同一帧里 `Ball` 和 `PenaltyPoint` 在图像中非常接近，或者有明显重叠
- 并且 `Ball` 置信度并没有显著弱于 `PenaltyPoint`
- 就优先保留 `Ball`，压掉这个 `PenaltyPoint`

这项规则是纯几何 + 硬阈值：

- 看中心距
- 看 IoU
- 看两者置信度差

这项改动属于：

- `视觉识别增强`

它与地面拟合无关，是为了解决“球被识别成点球点”这种类别冲突。

## 如何用这轮改动继续定位问题

针对你说的第二条“静止 2 秒内还会有正负 `10` 到 `20 cm` 跳动”，下一轮先不要只看结果值，要同时看 rerun 标签中的模式。

判断顺序：

1. 如果跳动帧大多是 `Ball[refined_plane]`
   说明主问题还在局部平面拟合本身，下一步应继续调 ROI 和 plane threshold。
2. 如果跳动帧经常是 `Ball[hold_last_valid]`
   说明拟合时断时续，问题偏向地面点不够、候选点被过滤过多、或 RANSAC 门限太严。
3. 如果跳动帧经常是 `Ball[fallback_z0]`
   说明这些帧本质上已经退回旧投影链路，抖动主要来自 bbox 底中点、头姿时间差、相机外参和固定地面假设。

## 和现有文档的关系

建议配合阅读：

- [足球定位校准说明](./ball_localization_calibration.md)
- [Brain 15 天冲刺规划](./brain_fast_iteration_plan.md)
- [参数与修改注意事项](./configuration.md)

其中：

- `ball_localization_calibration.md` 讲的是视觉投影链路怎么工作
- 本文讲的是误差怎么层层累积，以及先改哪里 ROI 最高
- `brain_fast_iteration_plan.md` 讲的是多人协作与 `brain` 快速拆模块
