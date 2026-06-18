# Ball Localization Calibration

## Overview

This document explains the ball localization path that is actually used by `brain`, the new ground-plane calibration logic added in `vision`, and the parameters that matter when tuning initial ball distance accuracy.

The concrete issue addressed here is the startup bias where the measured ball `x` is around `1.71 m` while the real distance is `1.65 m`. That is about `3.6%` error. The target is to reduce this to about `1%`, which means keeping the absolute error within roughly `0.0165 m`.

## Actual Data Path

The relevant code path is:

1. `vision` detects the ball bounding box.
2. `BallPoseEstimator` computes:
   - `position_projection`: the projected ground position
   - `position`: the depth-based 3D ball center
3. `vision_node` publishes both into `vision_interface::msg::DetectedObject`.
4. `brain` reads `position_projection` for ball position.

Key code references:

- [src/vision/src/vision_node.cpp](../src/vision/src/vision_node.cpp) line 386
- [src/vision/src/vision_node.cpp](../src/vision/src/vision_node.cpp) line 394
- [src/brain/src/brain.cpp](../src/brain/src/brain.cpp) line 2064

Important conclusion:

- The ball `x/y` that `brain` uses does not come from the depth sphere fit.
- Improving only `EstimateByDepth()` is not enough.
- The useful fix must improve `position_projection`.

## Previous Logic

Before this change, ball projection used a fixed ground assumption:

- take the bottom-center pixel of the ball box
- back-project a ray from camera intrinsics
- intersect that ray with the base-frame plane `z = 0`

The core function is:

- [src/vision/src/pose_estimator/pose_estimator.cpp](../src/vision/src/pose_estimator/pose_estimator.cpp) line 8

This is simple and stable, but any small error in:

- camera extrinsics
- robot head pose
- startup body posture
- bottom-point selection

will turn directly into a systematic distance bias.

## New Ground-Plane Calibration Logic

The new implementation keeps the old fixed-plane solution as fallback, but adds a depth-assisted projection path for balls.

Main idea:

1. Start from the ball box bottom-center pixel as before.
2. Build a local ROI around the ball, especially extending left/right and downward.
3. Exclude the ball box itself so the ball surface is not fitted as ground.
4. Transform ROI depth points into the `base` frame first, then keep only points whose `base.z` stays near the expected ground band.
5. Downsample the candidate points.
6. Fit a plane with `PlaneFitting()`.
7. If the fitted plane quality is good enough, treat that plane as the real ground plane.
8. Intersect the ball bottom ray with the fitted plane.
9. If any condition fails, fall back to the original `z = 0` plane intersection.

Code references:

- [src/vision/include/booster_vision/pose_estimator/pose_estimator.h](../src/vision/include/booster_vision/pose_estimator/pose_estimator.h) line 32
- [src/vision/src/pose_estimator/pose_estimator.cpp](../src/vision/src/pose_estimator/pose_estimator.cpp) line 13
- [src/vision/src/pose_estimator/pose_estimator.cpp](../src/vision/src/pose_estimator/pose_estimator.cpp) line 108

## What Changed In Code

### 1. Added `EstimateProjection()`

`PoseEstimator` now has a dedicated projection entry:

- default behavior: same as old `EstimateByColor()`
- ball-specific behavior: can use depth-assisted ground-plane refinement

This avoids changing `brain` or the message format.

### 2. `vision_node` now fills `position_projection` through the new entry

Instead of always calling `EstimateByColor()` for projection output, `vision_node` now calls `EstimateProjection()`:

- for balls: projection can be refined by depth ground fitting
- for all other classes: behavior stays the same

References:

- [src/vision/src/vision_node.cpp](../src/vision/src/vision_node.cpp) line 386
- [src/vision/src/vision_node.cpp](../src/vision/src/vision_node.cpp) line 463

### 3. Ball parameters were moved into the header

Defaults are now declared in the class definition:

- easier to see supported parameters in one place
- easier to maintain common defaults
- YAML can still override them at runtime

Reference:

- [src/vision/include/booster_vision/pose_estimator/pose_estimator.h](../src/vision/include/booster_vision/pose_estimator/pose_estimator.h) line 45

## Parameter Reference

The parameters live under `ball_pose_estimator` in:

- [src/vision/config/vision.yaml](../src/vision/config/vision.yaml) line 57

### Projection calibration parameters

`projection_use_ground_plane`

- Enables local ground-plane fitting for ball projection.
- Set to `false` to fully revert to the old fixed `z = 0` behavior.

`projection_roi_expand_x_ratio`

- Expands the ROI to the left and right by `bbox.width * ratio`.
- Too small: not enough ground points.
- Too large: may include nearby obstacles or noisy regions.

`projection_roi_expand_up_ratio`

- Expands the ROI upward.
- Usually should stay small, because upper pixels are less likely to be ground.

`projection_roi_expand_down_ratio`

- Expands the ROI downward by `bbox.height * ratio`.
- This is one of the most important parameters for finding enough ground points around the ball.

`projection_exclude_ball_padding_ratio`

- Expands the exclusion region around the ball box before plane fitting.
- Prevents the ball surface from contaminating the ground model.

`projection_downsample_leaf_size`

- Voxel size for downsampling the local ground cloud.
- Larger value: faster and smoother, but may lose detail.
- Smaller value: more detail, but noisier and heavier.

`projection_plane_fitting_distance_threshold`

- RANSAC inlier threshold for plane fitting.
- Larger value tolerates noise but may overfit non-ground points.
- Smaller value is stricter but may fail more often.

`projection_plane_confidence_threshold`

- Minimum inlier ratio for accepting the plane.
- If too high, the system falls back too often.
- If too low, unstable planes may be accepted.

`projection_plane_normal_z_min`

- Minimum `|nz|` ratio of the fitted plane normal.
- Used to reject planes that are too tilted to be treated as ground.

`projection_ground_max_abs_z`

- Allowed `|base.z|` band for ground candidates after transforming points from camera frame into base frame.
- Too small: valid ground points get filtered out.
- Too large: non-ground points can enter the fit.

`projection_min_points`

- Minimum number of candidate or downsampled points needed before fitting.
- If this is not met, the algorithm falls back to the fixed plane.

### Depth sphere-fit parameters

These parameters affect `position`, not the projection value consumed by `brain`:

`use_depth`

- Enables depth sphere fitting.
- If enabled for balls, invalid depth fit can still filter out false detections.

`radius`

- Expected ball radius in meters.

`down_sample_leaf_size`

- Downsampling size for ball-cluster point cloud.

`cluster_distance_threshold`

- Euclidean clustering threshold for the ball point cloud.

`fitting_distance_threshold`

- Inlier distance threshold for sphere fitting.

`minimum_cluster_size`

- Minimum size of a candidate cluster.

`filter_distance`

- If the projected ball is farther than this threshold, depth sphere fitting is skipped and projection is used directly.

`check_ball_height`

- Rejects fitted balls whose recovered height is clearly unreasonable.

## Tuning Guidance

### Suggested tuning order

1. Verify the system is actually using refined projection.
2. Check whether enough local ground points exist around the ball.
3. Tune plane acceptance thresholds.
4. Tune ROI size only after the previous steps are stable.
5. Recheck camera extrinsics if the residual bias stays systematic.

### Practical symptoms and likely causes

If measured distance is still too large:

- `projection_roi_expand_down_ratio` may be too small.
- `projection_plane_confidence_threshold` may be too strict, causing fallback.
- `projection_ground_max_abs_z` may be filtering out valid ground points.
- camera extrinsic `z/pitch` may still have residual calibration error.

If the result jumps or becomes unstable:

- `projection_roi_expand_x_ratio` may be too large.
- `projection_plane_confidence_threshold` may be too low.
- `projection_plane_normal_z_min` may be too permissive.
- the local depth image may include legs, edges, or field-border clutter.

If the refined path almost never activates:

- depth data near the ball is too sparse
- `projection_min_points` is too large
- `projection_plane_fitting_distance_threshold` is too small
- `projection_plane_confidence_threshold` is too high

## Limits And Expectations

Treating the fitted ground as the real ground can reduce startup bias, but it is not a replacement for calibration.

To reach `< 1%` distance error reliably, the following also matter:

- camera extrinsics must already be close
- ball bottom-center pixel should be consistent
- depth around the ball must be reasonably clean
- robot startup head pose should be stable

This change is best understood as a bias-reduction layer on top of the existing calibration chain.

## 中文参数说明

下面这些参数都在 `ball_pose_estimator` 下。要区分两组看。

### 一组影响 `position_projection`

这一组影响 `brain` 当前真正使用的球位置。

`projection_use_ground_plane`

- 功能：是否启用“局部地面拟合后再投影”。
- 影响：`true` 时新算法生效；`false` 时完全退回旧逻辑。
- 调参建议：现场对比时先开着，若怀疑新拟合不稳，可临时关掉做 A/B 对比。

`projection_roi_expand_x_ratio`

- 功能：球框左右扩展比例，基于球框宽度。
- 影响：决定左右方向采多少地面点。
- 太小：地面点不够，拟合容易失败。
- 太大：容易把旁边障碍物、腿、边线附近杂点也带进来，结果发飘。

`projection_roi_expand_up_ratio`

- 功能：球框上方扩展比例。
- 影响：通常不是核心参数，只是补一点上下文。
- 太大：容易把非地面区域带进来。
- 建议：保持小值。

`projection_roi_expand_down_ratio`

- 功能：球框下方扩展比例。
- 影响：这是最关键的一个 ROI 参数，决定球前后附近能拿到多少地面点。
- 太小：拟合经常失败或回退。
- 太大：会混入更远处地形、边界或杂物。
- 你这个“测远了”的场景，通常优先看这个参数。

`projection_exclude_ball_padding_ratio`

- 功能：在球框外再扩一圈，作为“禁入区”，这些点不参与地面拟合。
- 影响：防止球面点被当地面。
- 太小：球面污染平面。
- 太大：候选地面点变少。

`projection_downsample_leaf_size`

- 功能：局部地面点云的体素下采样尺寸。
- 影响：平衡稳定性和细节。
- 太小：点多但噪声也多，计算更重。
- 太大：拟合更平滑，但细节损失，可能把局部真实起伏抹平。

`projection_plane_fitting_distance_threshold`

- 功能：地面平面 RANSAC 的内点距离阈值。
- 影响：决定平面拟合对噪声的容忍度。
- 太小：稍微有噪声就拟合失败。
- 太大：不属于地面的点也可能被收进来。

`projection_plane_confidence_threshold`

- 功能：接受拟合平面的最低置信度阈值，本质上看内点占比。
- 影响：控制“拟合成功”的严格程度。
- 太高：经常回退到旧算法。
- 太低：错误平面也可能被接受，数值会跳。
- 如果你发现新逻辑几乎没起作用，先看这个。

`projection_plane_normal_z_min`

- 功能：要求平面法向在 z 方向上至少有多“像地面”。
- 影响：用来排除明显不是水平地面的平面。
- 太高：稍微有倾斜就不接受。
- 太低：墙、腿、斜面都可能混进来。

`projection_ground_max_abs_z`

- 功能：候选地面点允许的最大 `|base.z|`。
- 影响：这是地面带宽过滤，先把明显离地太高或太低的点去掉。
- 太小：真实地面点都被过滤掉，新逻辑经常回退。
- 太大：球体、腿、边缘点会混进来。
- 这里我特别改成了按 `base` 坐标系的 `z` 判断，不是按相机深度 `z`，否则语义是错的。

`projection_min_points`

- 功能：最少需要多少候选点/下采样点才允许做平面拟合。
- 影响：控制拟合是否有足够样本。
- 太大：新逻辑很少触发。
- 太小：少量噪点也可能拟合出“假地面”。

### 另一组影响 `position`

这一组是深度球心拟合参数，主要影响 `position`，不是 `brain` 当前直接用的 `position_projection`。

`use_depth`

- 功能：是否启用球的深度球面拟合。
- 影响：开启后会尝试根据深度点拟合球心。
- 注意：它主要影响 `position`，不是 `brain` 当前用的 `x/y` 主来源；但开启后如果深度估计失败，视觉侧还可能直接过滤掉这个球。

`radius`

- 功能：球半径，单位米。
- 影响：球面拟合时作为期望半径。
- 设错了会导致球心拟合偏差或者直接拟合失败。

`down_sample_leaf_size`

- 功能：球点云的下采样尺寸。
- 影响：和上面的地面下采样类似，但这是用于球点云，不是地面点云。

`cluster_distance_threshold`

- 功能：球点云聚类距离阈值。
- 影响：决定球点是否能聚成一团。
- 太小：一个球被拆成多个小簇。
- 太大：背景点也可能被并进球簇。

`fitting_distance_threshold`

- 功能：球面拟合的内点距离阈值。
- 影响：决定球面拟合对噪声的容忍度。
- 太小：拟合失败率高。
- 太大：非球面点也被收进去。

`minimum_cluster_size`

- 功能：最小聚类点数。
- 影响：控制多大一个簇才被认为值得尝试拟合成球。
- 太大：远处小球或稀疏点云容易被忽略。
- 太小：误检簇也可能进入拟合。

`filter_distance`

- 功能：如果球的投影距离已经超过这个值，就不做近距离深度球面拟合，直接返回投影结果。
- 影响：避免对远球做不可靠的深度拟合。
- 太小：很多球都不会做球面拟合。
- 太大：远距离噪声深度也会参与。

`check_ball_height`

- 功能：检查拟合出来的球心高度是否异常。
- 影响：防止把悬空噪点或误检物体当球。
- 开启后更保守，关闭后更宽松。

## Files To Check When Adjusting This Again

- [src/vision/include/booster_vision/pose_estimator/pose_estimator.h](../src/vision/include/booster_vision/pose_estimator/pose_estimator.h) line 1
- [src/vision/src/pose_estimator/pose_estimator.cpp](../src/vision/src/pose_estimator/pose_estimator.cpp) line 1
- [src/vision/src/vision_node.cpp](../src/vision/src/vision_node.cpp) line 386
- [src/brain/src/brain.cpp](../src/brain/src/brain.cpp) line 2022
- [src/vision/config/vision.yaml](../src/vision/config/vision.yaml) line 57
