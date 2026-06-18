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

- [src/vision/src/vision_node.cpp](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/vision/src/vision_node.cpp:386)
- [src/vision/src/vision_node.cpp](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/vision/src/vision_node.cpp:394)
- [src/brain/src/brain.cpp](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/brain/src/brain.cpp:2064)

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

- [src/vision/src/pose_estimator/pose_estimator.cpp](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/vision/src/pose_estimator/pose_estimator.cpp:8)

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

- [src/vision/include/booster_vision/pose_estimator/pose_estimator.h](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/vision/include/booster_vision/pose_estimator/pose_estimator.h:32)
- [src/vision/src/pose_estimator/pose_estimator.cpp](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/vision/src/pose_estimator/pose_estimator.cpp:13)
- [src/vision/src/pose_estimator/pose_estimator.cpp](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/vision/src/pose_estimator/pose_estimator.cpp:108)

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

- [src/vision/src/vision_node.cpp](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/vision/src/vision_node.cpp:386)
- [src/vision/src/vision_node.cpp](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/vision/src/vision_node.cpp:463)

### 3. Ball parameters were moved into the header

Defaults are now declared in the class definition:

- easier to see supported parameters in one place
- easier to maintain common defaults
- YAML can still override them at runtime

Reference:

- [src/vision/include/booster_vision/pose_estimator/pose_estimator.h](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/vision/include/booster_vision/pose_estimator/pose_estimator.h:45)

## Parameter Reference

The parameters live under `ball_pose_estimator` in:

- [src/vision/config/vision.yaml](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/vision/config/vision.yaml:57)

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

## Files To Check When Adjusting This Again

- [src/vision/include/booster_vision/pose_estimator/pose_estimator.h](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/vision/include/booster_vision/pose_estimator/pose_estimator.h:1)
- [src/vision/src/pose_estimator/pose_estimator.cpp](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/vision/src/pose_estimator/pose_estimator.cpp:1)
- [src/vision/src/vision_node.cpp](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/vision/src/vision_node.cpp:386)
- [src/brain/src/brain.cpp](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/brain/src/brain.cpp:2022)
- [src/vision/config/vision.yaml](/F:/bian/cppproject/K1_5v5_Demo_1.5/src/vision/config/vision.yaml:57)
