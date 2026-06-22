#pragma once

#include "types.h"

class Brain;

namespace support_position_planner {

bool getPlayerFieldPose(const Brain *brain, int playerId, Pose2D &pose);

Pose2D calcSupportTargetAroundStriker(
    const Brain *brain,
    const Pose2D &strikerPose,
    const Pose2D &selfPose,
    const Point &ballPosToField,
    bool aggressiveAssist
);

Pose2D calcAssistTarget(
    const Brain *brain,
    int rank,
    int strikerId,
    const Pose2D &selfPose,
    const Point &ballPosToField,
    double distToGoalline
);

Point calcDefensiveBallReference(
    const Brain *brain,
    const Point &fallbackBallPosToField,
    double lookaheadSecs = 0.6,
    bool preferBreach = true
);

Point calcPassReceiveTarget(
    const Brain *brain,
    int receiverId,
    const Pose2D &fallbackReceiverPose,
    const Point &ballPosToField,
    double forwardOffset = 0.45
);

Pose2D calcOneTwoGoTarget(
    const Brain *brain,
    const Pose2D &passerPose,
    double forwardDist = 1.0
);

} // namespace support_position_planner
