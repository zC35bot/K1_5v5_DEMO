#pragma once

#include <memory>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include "booster_vision/base/intrin.h"
#include "booster_vision/base/pose.h"
#include "booster_vision/model//detector.h"

namespace booster_vision {

class PoseEstimator {
public:
    using Ptr = std::shared_ptr<PoseEstimator>;
    PoseEstimator(const Intrinsics &intr) :
        intr_(intr) {
    }
    ~PoseEstimator() = default;

    virtual void Init(const YAML::Node &node){};

    virtual Pose EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb);
    virtual Pose EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb, const cv::Mat &depth);
    virtual Pose EstimateProjection(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb, const cv::Mat &depth);

    bool use_depth_;
protected:
    Intrinsics intr_;
};

class BallPoseEstimator : public PoseEstimator {
public:
    BallPoseEstimator(const Intrinsics &intr) :
        PoseEstimator(intr) {
    }
    ~BallPoseEstimator() = default;

    void Init(const YAML::Node &node) override;
    Pose EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb) override;
    Pose EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb, const cv::Mat &depth) override;
    Pose EstimateProjection(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb, const cv::Mat &depth) override;

private:
    float radius_ = 0.109f;
    float downsample_leaf_size_ = 0.01f;
    float cluster_distance_threshold_ = 0.01f;
    float fitting_distance_threshold_ = 0.01f;
    float filter_distance_ = 1.0f;
    int minimum_cluster_size_ = 150;
    bool check_ball_height_ = false;

    bool projection_use_ground_plane_ = true;
    float projection_roi_expand_x_ratio_ = 0.6f;
    float projection_roi_expand_up_ratio_ = 0.2f;
    float projection_roi_expand_down_ratio_ = 0.8f;
    float projection_exclude_ball_padding_ratio_ = 0.05f;
    float projection_downsample_leaf_size_ = 0.01f;
    float projection_plane_fitting_distance_threshold_ = 0.01f;
    float projection_plane_confidence_threshold_ = 0.55f;
    float projection_plane_normal_z_min_ = 0.9f;
    float projection_ground_max_abs_z_ = 0.15f;
    int projection_min_points_ = 120;
};

class HumanLikePoseEstimator : public PoseEstimator {
public:
    HumanLikePoseEstimator(const Intrinsics &intr) :
        PoseEstimator(intr) {
    }
    ~HumanLikePoseEstimator() = default;

    void Init(const YAML::Node &node) override;
    Pose EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb) override;
    Pose EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb, const cv::Mat &depth) override;

private:
    float downsample_leaf_size_;
    float statistic_outlier_multiplier_;
    float fitting_distance_threshold_;
};

class FieldMarkerPoseEstimator : public PoseEstimator {
public:
    FieldMarkerPoseEstimator(const Intrinsics &intr) :
        PoseEstimator(intr) {
    }
    ~FieldMarkerPoseEstimator() = default;

    void Init(const YAML::Node &node) override;
    Pose EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb) override;

private:
    bool refine_;
};

cv::Point3f CalculatePositionByIntersection(const Pose &p_eye2base, const cv::Point2f target_uv, const Intrinsics &intr);
cv::Point3f CalculatePositionByIntersection(const Pose &p_eye2base, const cv::Point2f target_uv, const Intrinsics &intr,
                                            const std::vector<float> &plane_coeffs);

struct FieldLineSegment {
    std::vector<cv::Point> contour_2d_points;
    std::vector<cv::Point3f> contour_3d_points;
    std::vector<cv::Point3f> line_model;
    std::vector<cv::Point2f> end_points_2d;
    std::vector<cv::Point3f> end_points_3d;
    unsigned int inlier_count;
    float accu_distance;
    double area;
};

std::vector<FieldLineSegment> FitFieldLineSegments(const Pose &p_eye2base,
                                                   const Intrinsics &intr,
                                                   const std::vector<std::vector<cv::Point>> &contours,
                                                   const int &line_segment_area_threshold);
cv::Mat DrawFieldLineSegments(cv::Mat &img, const std::vector<FieldLineSegment> &segs);

} // namespace booster_vision
