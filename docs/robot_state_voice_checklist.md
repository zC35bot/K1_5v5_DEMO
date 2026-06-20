# Robot State Voice Checklist

本文用于后续拿到实体机器人后，验证“机器人状态转发到 rerun”与“状态变化语音播报”是否按预期工作。

相关实现入口：

- 主循环在 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:324) 先执行 `tree->tick()`，再执行状态日志与状态播报，因此 `decision` 不是上一拍旧值。
- 语音发送函数在 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1106)。
- 状态汇总、`rerun` 转发、状态播报映射在 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3035)。
- 声音配置默认在 [src/brain/config/config.yaml](../src/brain/config/config.yaml:139)。
- `launch` 会加载 [src/brain/config/config.yaml](../src/brain/config/config.yaml:139) 和可选的 [src/brain/launch/launch.py](../src/brain/launch/launch.py:34) 中指定的 `config_local.yaml`。

## 1. 测试前准备

- 确认 `sound.enable: true`，见 [src/brain/config/config.yaml](../src/brain/config/config.yaml:140)。
- 确认 `sound.sound_pack: "espeak"`，见 [src/brain/config/config.yaml](../src/brain/config/config.yaml:141)。
- 确认现场环境里有实际消费 `/speak` 话题的 TTS 节点或等效链路；当前 `brain` 只负责发布字符串，不直接在本进程内调用系统扬声器。
- 如需同时观察日志，打开 rerun viewer，并确保 `rerunLog.enable_tcp` 或 `rerunLog.enable_file` 按现场方式配置。
- 保证机器人能正常收到 GameController、视觉检测、里程计、低层状态等基础输入，否则多数状态不会自然切换。

### 1.1 本轮临时打开的诊断日志

以下日志是这轮为了排查“状态播报链路”和“状态到语音映射”临时加的，建议在实机验收完成后评估是否关闭：

- `debug/robot_state`：核心状态摘要，当前更偏正式功能，位置见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3063)。
- `debug/robot_state_speech`：状态到英文播报词的映射结果，仅用于核对播报文案，位置见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3088)。
- `debug/speak`：TTS 发送诊断，能看到 `published / publisher not found / config not compatible / cooldown in process / repeat not allowed`，位置见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1110)。

建议后续处理原则：

- 保留 `debug/robot_state`，因为它本身就是对外有价值的状态可视化。
- `debug/robot_state_speech` 在播报词确认稳定后可以关闭。
- `debug/speak` 在 TTS 链路确认稳定后可以关闭，避免长期写过细的诊断日志。
- 关闭前先确认 checklist 中所有“有声/无声/冷却/重复拦截”场景都已经测过一轮。

## 2. 快速验收标准

- `rerun` 中能持续看到 `debug/robot_state`。
- 机器人状态变化时，只播报一次，不会每个 tick 连续重复播报。
- 相同状态持续保持时，不重复播报。
- 不同状态在 2 秒以内快速切换时，允许受 `speak()` 冷却时间限制而丢掉部分中间播报，这是当前设计行为，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1113)。
- `debug/robot_state` 的文本状态与实际行为树动作一致，尤其是 `find / chase / adjust / kick` 四种核心状态。

## 3. 预期播报词

当前中文状态与英文播报词映射如下，代码见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3067)：

| 中文状态 | 预期英文播报 |
| --- | --- |
| `等待启动` | `waiting to start` |
| `手动模式` | `manual mode` |
| `入场定位中` | `entering field and localizing` |
| `罚下/重新入场中` | `penalized and reentering field` |
| `等待对方开球` | `waiting for opponent kickoff` |
| `守门待命中` | `goalkeeper guarding` |
| `正在找球` | `searching for the ball` |
| `已找到球` | `ball found` |
| `正在追球` | `chasing the ball` |
| `已追到球，正在调整` | `ball reached, adjusting` |
| `已追到球，正在踢球` | `ball reached, kicking` |
| `已追到球，正在传球` | `ball reached, passing` |
| `视觉踢球中` | `visual kick in progress` |
| `协防/辅助中` | `assisting` |
| `回撤守门中` | `retreating to defend goal` |
| `状态未知` | `state unknown` |

## 4. 测试项

### 4.1 配置与链路检查

- [ ] 启动 `brain` 后，控制台未出现和 sound/TTS 明显相关的报错。
- [ ] 确认 `/speak` 话题上确实有字符串发布。
- [ ] 若现场没有声音，先区分是 `brain` 没发，还是 TTS 节点没收/没播。
- [ ] 若 `rerun` 已打开，确认 `debug/robot_state` 路径存在。

### 4.2 启动态

- [ ] 上电后、进入正常比赛前，观察是否出现 `等待启动`。
- [ ] 若初始 `control_state == 0`，首次进入该状态时应播报 `waiting to start`。
- [ ] 若状态没变，则不应持续重复播报。

### 4.3 手柄控制状态

- [ ] 切到 `control_state == 1` 时，播报 `manual mode`。
- [ ] 切到 `control_state == 2` 时，播报 `entering field and localizing`。
- [ ] 切回自动比赛态时，不应继续停留在上述播报状态。

### 4.4 罚下与重新入场

- [ ] 进入罚下态时，`debug/robot_state` 变为 `罚下/重新入场中`。
- [ ] 首次进入该状态时，播报 `penalized and reentering field`。
- [ ] 从罚下恢复后，状态能继续切回比赛相关状态，而不是卡死在罚下态。

### 4.5 开球等待态

- [ ] 在 `wait_for_opponent_kickoff == true` 时，状态为 `等待对方开球`。
- [ ] 首次进入该状态时，播报 `waiting for opponent kickoff`。
- [ ] 对方开球后，该状态应退出。

### 4.6 找球与找到球

- [ ] 当 `decision == "find"` 时，状态为 `正在找球`。
- [ ] 首次进入该状态时，播报 `searching for the ball`。
- [ ] 当 `decision` 为空或不是核心动作，但 `ball_location_known || tm_ball_pos_reliable || ballDetected` 成立时，状态为 `已找到球`。
- [ ] 首次进入该状态时，播报 `ball found`。
- [ ] 验证“已找到球”不会覆盖更高优先级状态，如 `kick`、`adjust`、`chase`。

### 4.7 追球、调整、踢球、传球

- [ ] `decision == "chase"` 时，状态为 `正在追球`，播报 `chasing the ball`。
- [ ] `decision == "adjust"` 时，状态为 `已追到球，正在调整`，播报 `ball reached, adjusting`。
- [ ] `decision == "kick"` 时，状态为 `已追到球，正在踢球`，播报 `ball reached, kicking`。
- [ ] `decision == "cross"` 时，状态为 `已追到球，正在传球`，播报 `ball reached, passing`。
- [ ] 观察这几个状态切换时，播报顺序与实际动作是否一致。

### 4.8 视觉踢球与协防

- [ ] `decision == "auto_visual_kick"` 时，状态为 `视觉踢球中`，播报 `visual kick in progress`。
- [ ] `decision == "assist"` 时，状态为 `协防/辅助中`，播报 `assisting`。

### 4.9 守门员状态

- [ ] 守门员 `goalie_mode == "guard"` 时，状态为 `守门待命中`，播报 `goalkeeper guarding`。
- [ ] 守门员 `decision == "retreat"` 时，状态为 `回撤守门中`，播报 `retreating to defend goal`。
- [ ] 验证守门员 `guard` 与 `retreat` 的优先级符合预期：当前 `guard` 优先于 `decision` 分支。

### 4.10 健康播报不互相抢占

- [ ] 若摄像头或 gamecontroller 异常，原有健康播报仍能工作，代码见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3091)。
- [ ] 当刚发生状态播报时，健康播报可能因为同一 `speak()` 冷却被顺延或跳过，这是当前设计允许的行为。

### 4.11 临时诊断日志验收

- [ ] `debug/robot_state_speech` 能看到状态与英文播报词的对应关系。
- [ ] `debug/speak` 在正常播报时显示 `published`。
- [ ] 若现场没有声音，但 `debug/speak` 已经显示 `published`，则问题更可能在 `/speak` 的消费节点，不在 `brain`。
- [ ] 若 `debug/speak` 显示 `cooldown in process`，确认这次漏播是否符合 2 秒冷却预期。
- [ ] 若 `debug/speak` 显示 `repeat not allowed`，确认这次未播是否只是重复状态未变化。
- [ ] 本轮所有诊断都完成后，记录这两个临时日志是否可以关闭：`debug/robot_state_speech`、`debug/speak`。

### 4.12 视觉球定位联合测试

这一部分用于一起验证“球定位是否稳定”和“状态播报是否因为球定位抖动而误触发”。详细背景可参考 [ball_localization_calibration.md](./ball_localization_calibration.md:201) 和 [ball_error_budget_and_fix_plan.md](./ball_error_budget_and_fix_plan.md:58)。

- [ ] 在 rerun 的 `image/detection_boxes` 上观察球标签，确认能看到 `Ball[refined_plane] / Ball[hold_last_valid] / Ball[fallback_z0]` 之一，参考 [docs/rerun.md](./rerun.md:106) 和 [ball_localization_calibration.md](./ball_localization_calibration.md:201)。
- [ ] 在 rerun 的 `field/ball` 上观察球位置，确认球静止时位置基本稳定，参考 [docs/rerun.md](./rerun.md:73)。
- [ ] 静止球测试：机器人静止、球静止时，状态不应在 `正在找球` 和 `已找到球` 之间频繁来回抖动。
- [ ] 轻微头动或短时深度掉点时，允许偶发 `hold_last_valid`，但不应长期主要停留在该模式，参考 [ball_localization_calibration.md](./ball_localization_calibration.md:219)。
- [ ] 若大多数帧都落在 `fallback_z0`，记录下来；这通常说明当前仍主要走旧投影链路，需继续检查视觉侧地面拟合或深度可用性，参考 [ball_localization_calibration.md](./ball_localization_calibration.md:225)。
- [ ] 近距离球测试：把球放在机器人前方较近位置，确认 `field/ball` 距离合理，且状态能从 `已找到球` 自然进入 `正在追球 / 已追到球，正在调整`。
- [ ] 中距离球测试：把球放到中等距离，确认状态能进入 `正在追球`，且位置不会明显跳变。
- [ ] 远距离球测试：把球放到更远处，确认如果视觉还能稳定识别，状态至少应稳定在 `已找到球` 或 `正在追球`，而不是频繁回到 `正在找球`。
- [ ] 遮挡丢球测试：短时间遮挡球后，观察状态是否先依赖记忆保持，再在记忆失效后回到 `正在找球`。
- [ ] 重新出现测试：球重新进入视野后，状态应恢复到 `已找到球` 或更高优先级动作状态，不应长时间卡在 `正在找球`。
- [ ] 记录“状态切换错误”到底是由决策逻辑导致，还是由球定位跳变导致。必要时结合 `debug/robot_state`、`field/ball`、`image/detection_boxes` 三者一起看。

## 5. 现场建议记录项

- [ ] 记录每个状态切换时的真实场景触发条件。
- [ ] 记录 `rerun` 中 `debug/robot_state` 的文本是否和语音一致。
- [ ] 记录 `debug/robot_state_speech` 和 `debug/speak` 是否给出了足够的排障信息。
- [ ] 记录是否存在“应该播但没播”的情况，并标注当时距离上一次播报是否小于 2 秒。
- [ ] 记录是否存在“播报内容正确，但时机晚一拍”的情况。
- [ ] 记录是否存在“中文状态判断正确，但英文词不自然”的情况，后续可单独优化文案。
- [ ] 记录球标签主要落在哪种投影模式：`refined_plane / hold_last_valid / fallback_z0`。
- [ ] 记录静止球时 `field/ball` 是否明显抖动，以及抖动是否会直接诱发状态播报抖动。
- [ ] 在本轮验收结束后，明确记录是否关闭这两个临时诊断日志：`debug/robot_state_speech`、`debug/speak`。

## 6. 已知限制

- 当前 TTS 文案使用英文，不是中文语音。
- 当前 `speak()` 带 2 秒冷却，同一时间窗口内的连续状态跃迁可能不会全部播出，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1113)。
- 当前 `brain` 只发布 `/speak` 字符串，是否真正出声还依赖现场的 TTS 消费节点。
- 当前为了排查问题额外打开了 `debug/robot_state_speech` 和 `debug/speak`；这两个更偏临时诊断用途，实机验收完成后应评估是否关闭。
- 由于当前未在本地接触实体机器人，本清单尚未经过实机验证。
