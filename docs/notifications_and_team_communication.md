# Notifications And Team Communication

本文整理两件事：

1. 机器人当前会对外播报哪些通知。
2. 机器人之间当前会互相发送哪些内容。

所有链接都使用相对路径，并精确到代码行。

## 语音播报总览

- `/speak` 发布器在 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:213) 和 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:214) 创建。
- 真正执行 TTS 发送的是 `Brain::speak()`，它会检查 `sound.enable`、`sound.sound_pack == "espeak"`、2 秒冷却以及重复文本抑制，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1474)。
- 当前默认配置已经打开 `espeak` 语音链路，见 [src/brain/config/config.yaml](../src/brain/config/config.yaml:140) 和 [src/brain/config/config.yaml](../src/brain/config/config.yaml:141)。

## 当前会播报的通知

### 1. 机器人状态变化播报

状态变化检测在 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3500)，语音文本映射在 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1599)。

当前状态播报文本如下：

- `waiting to start`
- `manual mode`
- `entering field and localizing`
- `penalized and reentering field`
- `waiting for opponent kickoff`
- `goalkeeper guarding`
- `searching for the ball`
- `ball found`
- `chasing the ball`
- `ball reached, adjusting`
- `ball reached, kicking`
- `ball reached, passing`
- `visual kick in progress`
- `one-two go in progress`
- `assisting`
- `retreating to defend goal`
- `goalkeeper intercepting`
- `state unknown`

触发规则：

- `statusReport()` 每次执行时先计算当前 `robotStateCode`，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3509)。
- 如果当前状态文本和上次不同，就调用 `speak(robotStateSpeech)`，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3517) 到 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3526)。

### 2. 健康/链路状态播报

同样由 `statusReport()` 触发，模板在 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3531) 到 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3542)。

当前会播报的模板有：

- `Team<teamId> Player<playerId> <player_role> OK`
- `camera lost`
- `gamecontrol lost`
- `camera lostgamecontrol lost`

说明：

- 最后一条是当前代码的直接拼接结果，没有分隔符，来源见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3537) 到 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3540)。
- 只有在这次没有播报机器人状态变化时，才会继续尝试播报这条健康报告，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3542)。

### 3. 事件型固定播报

这些不是“状态机自动映射”，而是代码在特定事件下直接调用 `speak()`：

| 播报文本 | 触发条件 | 代码位置 |
|---|---|---|
| `i become goalie` | 收到旧式守门员交接命令，且目标就是自己 | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:620) 到 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:629) |
| `halt` | `Intercept` 节点被 halted 时 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2262) 到 [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2271) |
| `Entering Left` / `Entering Right` | 入场定位左右方案判定结果变化 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2595) 到 [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:2609) |
| `Trying to stand up` | 倒地后触发站起恢复 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:4034) 到 [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:4043) |
| `Switch to striker` / `Switch to goal_keeper` | 角色切换后新旧角色不同 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:4098) 到 [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:4105) |
| `Calibration started` | 自动标定开始 | [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:4173) 到 [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:4174) |
| `vx factor: %.2f` | 手柄在线调 `vxFactor` | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1650) 到 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1653) |
| `yaw offset: %.2f` | 手柄在线调 `yawOffset` | [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1660) 到 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1663) |

### 4. 行为树里的固定文本播报

`Speak` 行为树节点本身支持播报任意文本，定义在 [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:4352)。

当前主比赛树里实际写死的文本有：

- `manual`，见 [src/brain/behavior_trees/game.xml](../src/brain/behavior_trees/game.xml:15) 到 [src/brain/behavior_trees/game.xml:16](../src/brain/behavior_trees/game.xml:16)
- `locate`，见 [src/brain/behavior_trees/game.xml](../src/brain/behavior_trees/game.xml:25) 到 [src/brain/behavior_trees/game.xml:27](../src/brain/behavior_trees/game.xml:27)
- `auto`，见 [src/brain/behavior_trees/game.xml](../src/brain/behavior_trees/game.xml:34) 到 [src/brain/behavior_trees/game.xml:35](../src/brain/behavior_trees/game.xml:35)

## 非 TTS 的声效提示

这部分不是 `espeak` 语音，而是通过 `/play_sound` 发给外部 sound pack 播放器。

- `/play_sound` 发布器在 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:213) 创建。
- 发送函数是 `Brain::playSound()`，见 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:1450)。
- 当前运行中自动触发的 sound pack 声效主要在 [src/brain/src/brain.cpp](../src/brain/src/brain.cpp:3668) 到 [src/brain/src/brain.cpp:3686](../src/brain/src/brain.cpp:3686)，包括：
  - `<sound_pack>-ready`
  - `<sound_pack>-celebrate`
  - `<sound_pack>-regret`
  - `<sound_pack>-chase`
  - `<sound_pack>-adjust`
  - `<sound_pack>-kick`
- 另外 `PlaySound` 行为树节点支持任意 `sound` 字符串，定义在 [src/brain/src/brain_tree.cpp](../src/brain/src/brain_tree.cpp:4342)，示例见 [src/brain/behavior_trees/chase.xml](../src/brain/behavior_trees/chase.xml:12)、[src/brain/behavior_trees/chase.xml](../src/brain/behavior_trees/chase.xml:17)、[src/brain/behavior_trees/chase.xml](../src/brain/behavior_trees/chase.xml:18)。

## 机器人之间会发送什么

机器人之间当前有两类消息：

1. discovery 广播
2. teammate 状态单播

### 1. Discovery 广播

协议结构定义在 [src/brain/include/team_communication_msg.h](../src/brain/include/team_communication_msg.h:45)。

字段只有 4 个：

- `validation`
- `communicationId`
- `teamId`
- `playerId`

发送逻辑在 [src/brain/src/brain_communication.cpp](../src/brain/src/brain_communication.cpp:219) 到 [src/brain/src/brain_communication.cpp:244](../src/brain/src/brain_communication.cpp:244)。

频率常量在 [src/brain/include/brain_communication.h](../src/brain/include/brain_communication.h:54)，当前是：

- `BROADCAST_DISCOVERY_INTERVAL_MS = 1000`

作用：

- 主要用于让队友知道“这台机器人在网内、它的 `playerId` 是多少、IP 从哪里来”，然后建立后续单播地址表。

### 2. Teammate 状态单播

协议结构定义在 [src/brain/include/team_communication_msg.h](../src/brain/include/team_communication_msg.h:7) 到 [src/brain/include/team_communication_msg.h](../src/brain/include/team_communication_msg.h:42)。

发送逻辑在 [src/brain/src/brain_communication.cpp](../src/brain/src/brain_communication.cpp:346) 到 [src/brain/src/brain_communication.cpp:425](../src/brain/src/brain_communication.cpp:425)。

接收并回填到 `TMStatus` 的逻辑在 [src/brain/src/brain_communication.cpp](../src/brain/src/brain_communication.cpp:484) 到 [src/brain/src/brain_communication.cpp:610](../src/brain/src/brain_communication.cpp:610)。

频率常量在 [src/brain/include/brain_communication.h](../src/brain/include/brain_communication.h:85)，当前是：

- `UNICAST_INTERVAL_MS = 100`

当前发送字段可以按语义分成几组：

#### 身份与基础合法性

- `validation`
- `communicationId`
- `teamId`
- `playerId`

来源见 [src/brain/src/brain_communication.cpp](../src/brain/src/brain_communication.cpp:354) 到 [src/brain/src/brain_communication.cpp:357](../src/brain/src/brain_communication.cpp:357)。

#### 角色与是否在场

- `playerRole`
- `teamRole`
- `isAlive`
- `isFallen`
- `isLead`

来源见 [src/brain/src/brain_communication.cpp](../src/brain/src/brain_communication.cpp:358) 到 [src/brain/src/brain_communication.cpp:362](../src/brain/src/brain_communication.cpp:362)。

#### 球、定位和接球成本

- `ballDetected`
- `ballLocationKnown`
- `ballConfidence`
- `ballRange`
- `cost`
- `ballPosToField`
- `robotPoseToField`
- `kickDir`
- `thetaRb`
- `robotState`

来源见 [src/brain/src/brain_communication.cpp](../src/brain/src/brain_communication.cpp:363) 到 [src/brain/src/brain_communication.cpp:372](../src/brain/src/brain_communication.cpp:372)。

#### 守门员仲裁/角色分配

- `assignedStrikerId`
- `assignedSupporterId`
- `captainDecisionId`

来源见 [src/brain/src/brain_communication.cpp](../src/brain/src/brain_communication.cpp:373) 到 [src/brain/src/brain_communication.cpp:375](../src/brain/src/brain_communication.cpp:375)。

#### 旧式命令字段

- `cmdId`
- `cmd`

来源见 [src/brain/src/brain_communication.cpp](../src/brain/src/brain_communication.cpp:376) 到 [src/brain/src/brain_communication.cpp:377](../src/brain/src/brain_communication.cpp:377)。

这两个字段的语义注释在 [src/brain/include/team_communication_msg.h](../src/brain/include/team_communication_msg.h:31) 和 [src/brain/include/team_communication_msg.h](../src/brain/include/team_communication_msg.h:32)。

#### 常规传球 / 二过一协作字段

- `passInitiator`
- `passState`
- `passPartnerPlayerId`
- `passSequenceId`
- `passReceiveReady`
- `passTakeoverAck`
- `passOneTwoIntent`
- `passTargetPosToField`
- `oneTwoState`
- `oneTwoReturnTargetPosToField`

来源见 [src/brain/src/brain_communication.cpp](../src/brain/src/brain_communication.cpp:378) 到 [src/brain/src/brain_communication.cpp:387](../src/brain/src/brain_communication.cpp:387)。

这些字段在接收端会写回本地 `TMStatus`，见 [src/brain/src/brain_communication.cpp](../src/brain/src/brain_communication.cpp:594) 到 [src/brain/src/brain_communication.cpp:603](../src/brain/src/brain_communication.cpp:603)。

### 3. 接收端目前的限制

接收端要求收到的数据包长度必须严格等于 `sizeof(TeamCommunicationMsg)`，见 [src/brain/src/brain_communication.cpp](../src/brain/src/brain_communication.cpp:504)。

这意味着：

- 同版本机器人之间通信正常。
- 混版本时，只要 `TeamCommunicationMsg` 结构体大小不同，就会直接丢包。

## 一句话总结

- 对外“播报”主链路是 `/speak + espeak`，核心文本来自 `getRobotStateSpeech()`、`statusReport()` 和少数事件型 `speak()` 调用。
- 机器人间“互发”主链路是 `discovery 广播 + 100ms teammate 单播`，后者已经包含角色仲裁、球信息、robotState、旧 cmd，以及常规传球/二过一协作字段。
