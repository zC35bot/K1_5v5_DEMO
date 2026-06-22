#include "support_position_planner.h"

#include <algorithm>
#include <cmath>

#include "brain.h"
#include "utils/math.h"

namespace support_position_planner {

bool getPlayerFieldPose(const Brain *brain, int playerId, Pose2D &pose)
{
    if (playerId == brain->config->playerId) {
        pose = brain->data->robotPoseToField;
        return brain->data->tmImAlive;
    }

    int idx = playerId - 1;
    if (idx < 0 || idx >= HL_MAX_NUM_PLAYERS) return false;
    const auto &status = brain->data->tmStatus[idx];
    if (!status.isAlive || status.isFallen) return false;
    pose = status.robotPoseToField;
    return true;
}

Point calcDefensiveBallReference(
    const Brain *brain,
    const Point &fallbackBallPosToField,
    double lookaheadSecs,
    bool preferBreach
)
{
    Point ref = fallbackBallPosToField;
    if (!brain || !brain->data) return ref;

    if (preferBreach && brain->data->ballWillBreach) {
        ref.x = brain->data->ballBreachPoint.x;
        ref.y = brain->data->ballBreachPoint.y;
        ref.z = 0.0;
        return ref;
    }

    const auto &predictions = brain->data->predictedBallPos;
    if (predictions.empty()) return ref;

    const double stepSecs = std::max(1e-3, brain->data->ballPredictStepIntervalMsecs / 1000.0);
    const int lookaheadIdx = std::min(
        static_cast<int>(predictions.size()) - 1,
        std::max(0, static_cast<int>(std::round(lookaheadSecs / stepSecs)))
    );
    ref.x = predictions[lookaheadIdx][0];
    ref.y = predictions[lookaheadIdx][1];
    ref.z = 0.0;
    return ref;
}

Pose2D calcSupportTargetAroundStriker(
    const Brain *brain,
    const Pose2D &strikerPose,
    const Pose2D &selfPose,
    const Point &ballPosToField,
    bool aggressiveAssist
)
{
    Pose2D pose = strikerPose;
    const double supportLateralGap = aggressiveAssist ? 1.8 : 1.6;
    const double supportForwardOffset = aggressiveAssist ? 0.2 : 0.0;
    const double sideBias = selfPose.y - strikerPose.y;
    const int sideSign =
        std::fabs(sideBias) > 0.25
        ? (sideBias > 0.0 ? 1 : -1)
        : ((ballPosToField.y - strikerPose.y) >= 0.0 ? -1 : 1);

    pose.x = strikerPose.x + supportForwardOffset;
    pose.y = strikerPose.y + sideSign * supportLateralGap;
    pose.theta = std::atan2(ballPosToField.y - pose.y, ballPosToField.x - pose.x);
    return pose;
}

Pose2D calcAssistTarget(
    const Brain *brain,
    int rank,
    int strikerId,
    const Pose2D &selfPose,
    const Point &ballPosToField,
    double distToGoalline
)
{
    auto fd = brain->config->fieldDimensions;
    Pose2D targetPose = {0.0, 0.0, 0.0};
    const double ownGoalX = -fd.length / 2.0;
    const double oppGoalX = fd.length / 2.0;
    const bool aggressiveAssist = brain->data->liveCount >= 3;

    Pose2D strikerPose = selfPose;
    const bool hasAssignedStrikerPose =
        strikerId > 0 && getPlayerFieldPose(brain, strikerId, strikerPose);

    if ((rank == 0 || rank == 1) && hasAssignedStrikerPose) {
        targetPose = calcSupportTargetAroundStriker(
            brain,
            strikerPose,
            selfPose,
            ballPosToField,
            aggressiveAssist
        );
    } else if (rank == 0 || rank == 1) {
        targetPose.x = ballPosToField.x - 2.0;
        targetPose.x = std::max(targetPose.x, ownGoalX + distToGoalline);
        double denom = ballPosToField.x - ownGoalX;
        if (std::fabs(denom) < 1e-6) denom = denom >= 0 ? 1e-6 : -1e-6;
        targetPose.y = ballPosToField.y * (targetPose.x - ownGoalX) / denom;
    } else if (rank == 2) {
        targetPose.x = -fd.length / 2.0 + fd.penaltyAreaLength + 1.0;
        if (targetPose.x > ballPosToField.x) targetPose.x = ballPosToField.x - 1.0;
        double denom = ballPosToField.x - ownGoalX;
        if (std::fabs(denom) < 1e-6) denom = denom >= 0 ? 1e-6 : -1e-6;
        targetPose.y = ballPosToField.y * (targetPose.x - ownGoalX) / denom;
    } else if (rank == 3) {
        targetPose.x = -fd.length / 2.0 + fd.penaltyAreaLength;
        if (targetPose.x > ballPosToField.x) targetPose.x = ballPosToField.x - 0.5;
        double denom = ballPosToField.x - ownGoalX;
        if (std::fabs(denom) < 1e-6) denom = denom >= 0 ? 1e-6 : -1e-6;
        targetPose.y = ballPosToField.y * (targetPose.x - ownGoalX) / denom;
    } else {
        targetPose.x = -fd.length / 2.0 + fd.penaltyDist;
        if (targetPose.x > ballPosToField.x) targetPose.x = ballPosToField.x - 0.5;
        double denom = ballPosToField.x - ownGoalX;
        if (std::fabs(denom) < 1e-6) denom = denom >= 0 ? 1e-6 : -1e-6;
        targetPose.y = ballPosToField.y * (targetPose.x - ownGoalX) / denom;
    }

    targetPose.x = cap(targetPose.x, oppGoalX - fd.penaltyAreaLength - 0.2, ownGoalX + distToGoalline);
    targetPose.y = cap(targetPose.y, fd.width / 2.0 - 0.7, -fd.width / 2.0 + 0.7);
    targetPose.theta = std::atan2(ballPosToField.y - targetPose.y, ballPosToField.x - targetPose.x);
    return targetPose;
}

Point calcPassReceiveTarget(
    const Brain *brain,
    int receiverId,
    const Pose2D &fallbackReceiverPose,
    const Point &ballPosToField,
    double forwardOffset
)
{
    Pose2D receiverPose = fallbackReceiverPose;
    getPlayerFieldPose(brain, receiverId, receiverPose);

    auto fd = brain->config->fieldDimensions;
    Point target;
    target.x = std::max(receiverPose.x + forwardOffset, ballPosToField.x + 0.15);
    target.y = receiverPose.y;
    target.z = 0.0;
    target.x = cap(target.x, fd.length / 2.0 - 0.4, -fd.length / 2.0 + fd.goalAreaLength + 0.4);
    target.y = cap(target.y, fd.width / 2.0 - 0.6, -fd.width / 2.0 + 0.6);
    return target;
}

Pose2D calcOneTwoGoTarget(
    const Brain *brain,
    const Pose2D &passerPose,
    double forwardDist
)
{
    auto fd = brain->config->fieldDimensions;
    Pose2D targetPose = passerPose;
    targetPose.x += forwardDist;
    targetPose.x = cap(targetPose.x, fd.length / 2.0 - 0.5, -fd.length / 2.0 + fd.goalAreaLength + 0.5);
    targetPose.y = cap(targetPose.y, fd.width / 2.0 - 0.7, -fd.width / 2.0 + 0.7);
    targetPose.theta = std::atan2(-targetPose.y, fd.length / 2.0 - targetPose.x);
    return targetPose;
}

} // namespace support_position_planner
