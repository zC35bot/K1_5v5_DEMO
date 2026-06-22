#include "role_manager.h"

#include <algorithm>
#include <cmath>

#include "brain.h"

namespace role_manager {

namespace {

bool isParticipatingPassState(int passState, int oneTwoState) {
    return passState != PASS_STATE_IDLE || oneTwoState != ONE_TWO_STATE_IDLE;
}

} // namespace

double calcCaptainBallCost(const Brain *brain, int playerId, bool isSelf)
{
    const double lostBallPenalty = 100.0;
    const double fallenPenalty = 1000.0;
    const double notAlivePenalty = 5000.0;
    if (playerId <= 0) return 1e9;

    if (isSelf) {
        if (!brain->data->tmImAlive) return notAlivePenalty;
        if (brain->data->recoveryState == RobotRecoveryState::HAS_FALLEN) return fallenPenalty;
        if (!brain->tree->getEntry<bool>("ball_location_known")) return lostBallPenalty + brain->data->tmMyCost;
        return brain->data->tmMyCost;
    }

    int idx = playerId - 1;
    if (idx < 0 || idx >= HL_MAX_NUM_PLAYERS) return 1e9;
    const auto &status = brain->data->tmStatus[idx];
    if (!status.isAlive) return notAlivePenalty;
    if (status.isFallen) return fallenPenalty;
    if (status.robotState == ROBOT_STATE_FIND_BALL || !status.ballLocationKnown) return lostBallPenalty + status.cost;
    return status.cost;
}

std::vector<int> collectFrontfieldIds(
    const Brain *brain,
    const std::vector<int> &aliveTmIdxs,
    const std::string &selfRole
)
{
    std::vector<int> ids;
    const int selfId = brain->config->playerId;
    if (selfRole != "goal_keeper" && brain->data->tmImAlive) ids.push_back(selfId);
    for (int idx : aliveTmIdxs) {
        const auto &tmStatus = brain->data->tmStatus[idx];
        if (tmStatus.role == "goal_keeper") continue;
        ids.push_back(idx + 1);
    }
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    return ids;
}

bool isFrontfieldId(const std::vector<int> &frontfieldIds, int playerId)
{
    return std::find(frontfieldIds.begin(), frontfieldIds.end(), playerId) != frontfieldIds.end();
}

CaptainAssignment chooseCaptainAssignment(
    const Brain *brain,
    const std::vector<int> &frontfieldIds,
    const CaptainAssignment &currentAssignment,
    double stealMargin
)
{
    CaptainAssignment nextAssignment;
    int bestStrikerId = 0;
    double bestStrikerCost = 1e9;
    const int selfId = brain->config->playerId;

    for (int playerId : frontfieldIds) {
        double candidateCost = calcCaptainBallCost(brain, playerId, playerId == selfId);
        if (
            candidateCost < bestStrikerCost - 1e-6
            || (std::fabs(candidateCost - bestStrikerCost) < 1e-6 && (bestStrikerId == 0 || playerId < bestStrikerId))
        ) {
            bestStrikerCost = candidateCost;
            bestStrikerId = playerId;
        }
    }

    nextAssignment.strikerId = bestStrikerId;
    if (isFrontfieldId(frontfieldIds, currentAssignment.strikerId) && currentAssignment.strikerId > 0) {
        double currentCost = calcCaptainBallCost(brain, currentAssignment.strikerId, currentAssignment.strikerId == selfId);
        double challengerCost = calcCaptainBallCost(brain, bestStrikerId, bestStrikerId == selfId);
        if (challengerCost > currentCost - stealMargin) {
            nextAssignment.strikerId = currentAssignment.strikerId;
        }
    }

    double bestSupportCost = 1e9;
    for (int playerId : frontfieldIds) {
        if (playerId == nextAssignment.strikerId) continue;
        double candidateCost = calcCaptainBallCost(brain, playerId, playerId == selfId);
        if (
            candidateCost < bestSupportCost - 1e-6
            || (std::fabs(candidateCost - bestSupportCost) < 1e-6 && (nextAssignment.supporterId == 0 || playerId < nextAssignment.supporterId))
        ) {
            bestSupportCost = candidateCost;
            nextAssignment.supporterId = playerId;
        }
    }

    if (nextAssignment.supporterId == nextAssignment.strikerId) {
        nextAssignment.supporterId = 0;
    }
    if (nextAssignment.supporterId == 0 && frontfieldIds.size() >= 2) {
        for (int playerId : frontfieldIds) {
            if (playerId != nextAssignment.strikerId) {
                nextAssignment.supporterId = playerId;
                break;
            }
        }
    }

    return nextAssignment;
}

ActivePassContext findActivePassContext(const Brain *brain)
{
    ActivePassContext ctx;
    auto consider = [&](int ownerId,
                        bool initiator,
                        int passState,
                        int partnerId,
                        int sequenceId,
                        bool oneTwoIntent,
                        int oneTwoState,
                        const Point &passTarget,
                        const Point &oneTwoReturnTarget) {
        if (!initiator) return;
        if (!isParticipatingPassState(passState, oneTwoState)) return;
        if (sequenceId <= 0) return;
        if (
            !ctx.valid
            || sequenceId > ctx.sequenceId
            || (sequenceId == ctx.sequenceId && ownerId < ctx.ownerId)
        ) {
            ctx.valid = true;
            ctx.ownerId = ownerId;
            ctx.partnerId = partnerId;
            ctx.sequenceId = sequenceId;
            ctx.passState = passState;
            ctx.oneTwoState = oneTwoState;
            ctx.oneTwoIntent = oneTwoIntent;
            ctx.passTargetPosToField = passTarget;
            ctx.oneTwoReturnTargetPosToField = oneTwoReturnTarget;
        }
    };

    consider(
        brain->config->playerId,
        brain->data->tmMyPassInitiator,
        brain->data->tmMyPassState,
        brain->data->tmMyPassPartnerPlayerId,
        brain->data->tmMyPassSequenceId,
        brain->data->tmMyPassOneTwoIntent,
        brain->data->tmMyOneTwoState,
        brain->data->tmMyPassTargetPosToField,
        brain->data->tmMyOneTwoReturnTargetPosToField
    );

    for (int i = 0; i < HL_MAX_NUM_PLAYERS; ++i) {
        const auto &status = brain->data->tmStatus[i];
        if (!status.isAlive) continue;
        if (brain->msecsSince(status.timeLastCom) > 1500.0) continue;
        consider(
            i + 1,
            status.passInitiator,
            status.passState,
            status.passPartnerPlayerId,
            status.passSequenceId,
            status.passOneTwoIntent,
            status.oneTwoState,
            status.passTargetPosToField,
            status.oneTwoReturnTargetPosToField
        );
    }

    return ctx;
}

bool getPassPartnerStatus(
    const Brain *brain,
    int playerId,
    int ownerId,
    int expectedSeq,
    bool &receiveReady,
    bool &takeoverAck
)
{
    receiveReady = false;
    takeoverAck = false;
    if (playerId <= 0 || expectedSeq <= 0) return false;

    if (playerId == brain->config->playerId) {
        if (
            brain->data->tmMyPassSequenceId != expectedSeq
            || brain->data->tmMyPassPartnerPlayerId != ownerId
        ) {
            return false;
        }
        receiveReady = brain->data->tmMyPassReceiveReady;
        takeoverAck = brain->data->tmMyPassTakeoverAck;
        return true;
    }

    int idx = playerId - 1;
    if (idx < 0 || idx >= HL_MAX_NUM_PLAYERS) return false;
    const auto &status = brain->data->tmStatus[idx];
    if (!status.isAlive) return false;
    if (brain->msecsSince(status.timeLastCom) > 1500.0) return false;
    if (
        status.passSequenceId != expectedSeq
        || status.passPartnerPlayerId != ownerId
    ) {
        return false;
    }
    receiveReady = status.passReceiveReady;
    takeoverAck = status.passTakeoverAck;
    return true;
}

} // namespace role_manager
