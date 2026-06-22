#pragma once

#include <string>
#include <vector>

#include "types.h"

class Brain;

namespace role_manager {

struct CaptainAssignment {
    int strikerId = 0;
    int supporterId = 0;
};

struct ActivePassContext {
    bool valid = false;
    int ownerId = 0;
    int partnerId = 0;
    int sequenceId = 0;
    int passState = PASS_STATE_IDLE;
    int oneTwoState = ONE_TWO_STATE_IDLE;
    bool oneTwoIntent = false;
    Point passTargetPosToField;
    Point oneTwoReturnTargetPosToField;
};

double calcCaptainBallCost(const Brain *brain, int playerId, bool isSelf);

std::vector<int> collectFrontfieldIds(
    const Brain *brain,
    const std::vector<int> &aliveTmIdxs,
    const std::string &selfRole
);

bool isFrontfieldId(const std::vector<int> &frontfieldIds, int playerId);

CaptainAssignment chooseCaptainAssignment(
    const Brain *brain,
    const std::vector<int> &frontfieldIds,
    const CaptainAssignment &currentAssignment,
    double stealMargin
);

ActivePassContext findActivePassContext(const Brain *brain);

bool getPassPartnerStatus(
    const Brain *brain,
    int playerId,
    int ownerId,
    int expectedSeq,
    bool &receiveReady,
    bool &takeoverAck
);

} // namespace role_manager
