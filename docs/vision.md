# Vision 模块说明

## 1. 入口与初始化

入口文件：

- `src/vision/src/main.cpp`
- `src/vision/src/vision_node.cpp`

启动方式：

- `ros2 launch vision launch.py ...`

`main.cpp` 会：

1. 读取命令行参数中的配置文件路径
2. 创建 `VisionNode`
3. 调用 `Init(config_template_path, config_path)`
4. 用 `MultiThreadedExecutor(4)` 跑节点

## 2. 配置加载方式

`VisionNode::Init()` 的配置加载顺序是：

1. 读取模板配置 `vision.yaml`
2. 如果第二个文件存在，则把它 merge 进模板配置
3. 再读取 launch 参数覆盖运行时开关

当前 launch 参数可覆盖：

- `offline_mode`
- `show_det`
- `show_seg`
- `save_data`
- `save_depth`
- `save_fps`
- `detection_model_path`
- `segmentation_model_path`
- `camera_type`

模型路径覆盖逻辑：

- 如果 launch 传的是空字符串，则使用 `vision.yaml` 里的 `model_path`
- 如果传的是非空相对路径，会拼到 `vision` 包 share 目录下
- 如果传的是绝对路径，则直接使用

## 3. 运行主链路

### 3.1 输入

`vision_node` 主要订阅：

- 彩色图像 `/boostercamera/head/rgb`
- 深度图 `/boostercamera/head/depth`，当 `use_depth=true` 时
- 头部位姿 `/head_pose`
- 动态标定补偿 `/booster_vision/cal_param`

离线模式下：

- 不直接订阅 `/head_pose`
- 改为订阅 `/booster_vision/t_head2base`

### 3.2 输出

主要发布：

- `/booster_vision/detection`
- `/booster_vision/line_segments`
- `/booster_vision/ball`
- `/booster_vision/t_head2base`

其中：

- `detection` 给 `brain` 做目标状态更新
- `line_segments` 给 `brain` 做场线定位和出界判断
- `ball` 是单独给球的简化消息

## 4. 视觉主流程

### 4.1 检测流程 `ColorCallback -> ProcessData`

主流程：

1. ROS 图像转 `cv::Mat`
2. 通过 `DataSyncer` 与深度和头部位姿同步
3. 根据 `p_head2base * p_headprime2head * p_eye2head` 求当前相机到机体坐标变换
4. 调检测器推理
5. 按类别选择不同的 `PoseEstimator`
6. 估计目标位置
7. 组装 `vision_interface::msg::Detections`
8. 发布检测结果和单独球消息

### 4.2 分割流程 `SegmentationCallback -> ProcessSegmentationData`

主流程：

1. ROS 图像转 `cv::Mat`
2. 与头部位姿同步
3. 调分割器推理
4. 从掩膜轮廓拟合出场线段
5. 发布 `/booster_vision/line_segments`

### 4.3 深度流程 `DepthCallback`

只负责把深度图放进 `DataSyncer`，供目标位置估计时取同步深度。

## 5. 位姿与补偿

视觉位置估计中的相机姿态是三段乘积：

```text
p_eye2base = p_head2base * p_headprime2head * p_eye2head
```

其中：

- `p_head2base`：来自 `/head_pose`
- `p_eye2head`：相机外参，来自 `vision.yaml`
- `p_headprime2head`：动态补偿项，由 pitch/yaw/z compensation 构成

`CalParamCallback` 会更新：

- `pitch_compensation`
- `yaw_compensation`
- `z_compensation`

这个接口是给 Brain 侧自动视觉标定节点动态调整用的。

## 6. 检测器与分割器

### 6.1 检测器

创建位置：

- `src/vision/src/model/detector.cc`

当前支持：

- TensorRT `.engine`
- ONNX，前提是构建时启用了 `ENABLE_ONNX`

当前默认类别顺序：

- `Ball`
- `Goalpost`
- `Person`
- `LCross`
- `TCross`
- `XCross`
- `PenaltyPoint`
- `Opponent`
- `BRMarker`

### 6.2 分割器

创建位置：

- `src/vision/src/model/segmentor.cc`

当前主要输出：

- `CircleLine`
- `Line`

## 7. 位置估计器

`VisionNode` 里根据类别自动挑选估计器：

- `Ball`：`BallPoseEstimator`
- `Person / Opponent / Goalpost`：`HumanLikePoseEstimator`
- `Cross / PenaltyPoint`：`FieldMarkerPoseEstimator`
- 其他：默认 `PoseEstimator`

这一步决定目标三维位置是按什么规则算出来的。

## 8. 数据保存

当 `save_data=true` 时：

- 会创建 `DataLogger`
- 保存同步后的原始数据
- 把最终合并后的 YAML 配置也落一份

保存路径根目录：

- `$HOME/Workspace/vision_log/<timestamp>/`

保存频率受 `save_fps` 控制，内部换算成：

- `save_every_n_frame = max(1, 30 / save_fps)`

注意：

- 当前代码默认按 30 FPS 估算采样间隔
- 并不会自适应真实相机帧率

## 9. 标定模块

文件：

- `src/vision/src/calibration/calibration_node.cpp`
- `src/vision/src/calibration/calibration.cpp`
- `src/vision/src/calibration/board_detector.cpp`

### 9.1 两种模式

支持两种模式：

- `handeye`
- `offset`

### 9.2 `handeye`

使用棋盘格：

1. 采样多帧图像和头部位姿
2. 检测棋盘角点
3. 计算 `camera -> head` 外参
4. 计算重投影误差
5. 可选择回写配置

### 9.3 `offset`

使用场地标志点：

1. 检测 `Cross / PenaltyPoint`
2. 与 `field.yaml` 中理论位置做匹配
3. 优化 `head' -> head` 小补偿
4. 回写到 `camera.extrin` 并清零补偿项

### 9.4 标定结果输出

代码会把结果写入：

- 日志目录中的 `vision_local.yaml.*`
- 用户确认后可覆盖输入配置
- 也可尝试写 `/opt/booster/vision.yaml`

如果系统目录写失败，会回退到：

- `/tmp/vision.yaml`

### 9.5 快速启动脚本

- `scripts/start_calibration.sh`：运行 `calibration_node handeye`

完成后还会尝试：

- `sudo cp /tmp/vision.yaml /opt/booster/vision.yaml`

## 10. 当前版本注意点

1. `scripts/start_vision.sh` 默认使用 `vision_config_path:=/opt/booster`，依赖外部部署文件。
2. `VisionNode::Init()` 中，彩色和深度订阅各被重复创建了一次，最终以后一次覆盖前一次为准，虽然通常能工作，但结构上有冗余。
3. `save_fps` 是按 30 FPS 假设换算的，不适合把它理解成严格的每秒保存帧数。
4. 相机 topic 目前对不同 `camera_type` 基本都用了同一组 `/boostercamera/head/*` 话题，类型字段更多是为后续兼容预留。
