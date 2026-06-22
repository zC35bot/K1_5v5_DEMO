#include <algorithm>
#include <iostream>
#include <string>
#include <fstream>  // 娣诲姞杩欎竴琛?
#include <yaml-cpp/yaml.h>  // 娣诲姞杩欎竴琛?

#include "brain.h"
#include "role_manager.h"
#include "utils/print.h"
#include "utils/math.h"
#include "utils/misc.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using std::placeholders::_1;

#define SUB_STATE_QUEUE_SIZE 1

Brain::Brain() : rclcpp::Node("brain_node")
{
    // 鍒濆鍖杢f骞挎挱鍣?
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // 瑕佹敞鎰忓弬鏁板繀椤诲厛鍦ㄨ繖閲屽０鏄庯紝鍚﹀垯绋嬪簭閲屼篃璇讳笉鍒?
    // 閰嶇疆鍦?yaml 鏂囦欢涓殑鍙傛暟锛屽鏋滄湁灞傜骇缁撴瀯锛岀敤鐐瑰垎鍙锋潵鑾峰彇

    declare_parameter<int>("game.team_id", 0);
    declare_parameter<int>("game.player_id", 29);
    declare_parameter<string>("game.field_type", "");

    declare_parameter<string>("game.player_role", "");
    declare_parameter<string>("game.player_start_pos", "left");
    declare_parameter<bool>("game.treat_person_as_robot", false);
    declare_parameter<int>("game.number_of_players", 2);

    declare_parameter<double>("robot.robot_height", 1.0);
    declare_parameter<double>("robot.odom_factor", 1.0);
    declare_parameter<double>("robot.vx_factor", 0.5);
    declare_parameter<double>("robot.yaw_offset", 0.0);
    declare_parameter<double>("robot.vx_limit", 1.0);
    declare_parameter<double>("robot.vy_limit", 0.4);
    declare_parameter<double>("robot.vtheta_limit", 1.0);

    declare_parameter<double>("strategy.ball_confidence_threshold", 50.0);
    declare_parameter<double>("strategy.ball_confidence_decay_rate", 3.0);
    declare_parameter<bool>("strategy.enable_stable_kick", false);
    declare_parameter<double>("strategy.ball_memory_timeout", 3.0);
    declare_parameter<double>("strategy.tm_ball_dist_threshold", 3.0);
    declare_parameter<bool>("strategy.limit_near_ball_speed", true);
    declare_parameter<double>("strategy.near_ball_speed_limit", 0.3);
    declare_parameter<double>("strategy.near_ball_range", 4.0);
    declare_parameter<bool>("strategy.soft_kickoff", false);
    declare_parameter<double>("strategy.soft_kickoff_speed", 0.3);
    declare_parameter<double>("strategy.kick_range", 1.0);
    declare_parameter<double>("strategy.kick_theta_range", 0.2);
    declare_parameter<bool>("strategy.abort_kick_when_ball_moved", false);
    declare_parameter<double>("strategy.abort_kick_ball_move_threshold", 0.3);
    declare_parameter<bool>("strategy.enable_bypass", false);
    declare_parameter<bool>("strategy.enable_shoot", false);
    declare_parameter<bool>("strategy.enable_directional_kick", false);

    declare_parameter<bool>("strategy.use_squat_block", false);
    declare_parameter<double>("strategy.squat_block_msecs", 2000.0);
    declare_parameter<bool>("strategy.use_move_block", true);
    declare_parameter<double>("strategy.move_block_msecs", 2000.0);
    declare_parameter<double>("strategy.goalie_guard_enter_ball_x", 0.0);
    declare_parameter<double>("strategy.goalie_guard_exit_ball_x", 0.3);
    declare_parameter<double>("strategy.goalie_intercept_enable_ball_x", -1.0);
    declare_parameter<double>("strategy.goalie_intercept_lead_secs", 3.0);
    declare_parameter<double>("strategy.goalie_guard_squat_enable_ball_x", -3.2);
    declare_parameter<double>("strategy.goalie_guard_squat_disable_ball_x", -2.6);
    declare_parameter<double>("strategy.goalie_guard_squat_enable_ball_speed_x", 0.2);
    declare_parameter<double>("strategy.goalie_guard_squat_ball_y_margin", 0.25);
    declare_parameter<double>("strategy.goalie_guard_squat_recover_pose_dist", 0.35);
    declare_parameter<bool>("strategy.enable_auto_visual_kick", false);
    declare_parameter<double>("strategy.auto_visual_kick_enable_dist_min", 0.2);
    declare_parameter<double>("strategy.auto_visual_kick_enable_dist_max", 4.0);
    declare_parameter<double>("strategy.auto_visual_kick_enable_angle", 0.8);
    declare_parameter<bool>("strategy.enable_auto_visual_defend", false);

    declare_parameter<double>("ball_predictor.step_interval", 100.0);
    declare_parameter<int>("ball_predictor.step_cnt", 50);
    declare_parameter<int>("ball_predictor.linear_steps", 5);
    declare_parameter<double>("ball_predictor.linear_rsq_threshold", 0.98);
    declare_parameter<double>("ball_predictor.acceleration", -0.4);

    declare_parameter<bool>("strategy.power_shoot.enable", false);
    declare_parameter<bool>("strategy.power_shoot.use_for_kickoff", false);
    declare_parameter<double>("strategy.power_shoot.xmin", 0.5);
    declare_parameter<double>("strategy.power_shoot.xmax", 1.0);
    declare_parameter<double>("strategy.power_shoot.ymin", -0.5);
    declare_parameter<double>("strategy.power_shoot.ymax", 0.5);
    declare_parameter<double>("strategy.shoot.threat_threshold", 0.0);
    declare_parameter<double>("strategy.shoot.xmin", 0.5);
    declare_parameter<double>("strategy.shoot.xmax", 1.0);
    declare_parameter<double>("strategy.shoot.ymin", -0.5);
    declare_parameter<double>("strategy.shoot.ymax", 0.5);
    declare_parameter<bool>("strategy.cooperation.enable_role_switch", true);
    declare_parameter<double>("strategy.cooperation.ball_control_cost_threshold", 10.0);

    declare_parameter<int>("obstacle_avoidance.depth_sample_step", 16);
    declare_parameter<double>("obstacle_avoidance.obstacle_min_height", 0.15);
    declare_parameter<double>("obstacle_avoidance.grid_size", 0.2);
    declare_parameter<double>("obstacle_avoidance.max_x", 0.2);
    declare_parameter<double>("obstacle_avoidance.max_y", 0.2);
    declare_parameter<double>("obstacle_avoidance.exclusion_x", 0.25);
    declare_parameter<double>("obstacle_avoidance.exclusion_y", 0.4);
    declare_parameter<double>("obstacle_avoidance.ball_exclusion_radius", 0.3);
    declare_parameter<double>("obstacle_avoidance.ball_exclusion_height", 0.3);
    declare_parameter<int>("obstacle_avoidance.occupancy_threshold", 5);
    declare_parameter<double>("obstacle_avoidance.collision_threshold", 0.5);
    declare_parameter<double>("obstacle_avoidance.safe_distance", 2.0);
    declare_parameter<double>("obstacle_avoidance.avoid_secs", 3.0);
    declare_parameter<bool>("obstacle_avoidance.enable_freekick_avoid", false);
    declare_parameter<double>("obstacle_avoidance.freekick_start_placing_safe_distance", 0.5);
    declare_parameter<double>("obstacle_avoidance.freekick_start_placing_avoid_secs", 1.5);
    declare_parameter<double>("obstacle_avoidance.obstacle_memory_msecs", 500.0);
    declare_parameter<bool>("obstacle_avoidance.avoid_during_chase", false);
    declare_parameter<double>("obstacle_avoidance.chase_ao_safe_dist", 2.0);
    declare_parameter<bool>("obstacle_avoidance.avoid_during_kick", false);
    declare_parameter<double>("obstacle_avoidance.kick_ao_safe_dist", 1.0);
    declare_parameter<bool>("obstacle_avoidance.kick_ao_use_shoot", false);
    declare_parameter<bool>("obstacle_avoidance.always_turn_left", false);

    declare_parameter<int>("locator.min_marker_count", 5);
    declare_parameter<double>("locator.max_residual", 0.3);

    declare_parameter<bool>("enable_com", false);

    declare_parameter<bool>("rerunLog.enable_tcp", false);
    declare_parameter<string>("rerunLog.server_ip", "");
    declare_parameter<bool>("rerunLog.enable_file", false);
    declare_parameter<string>("rerunLog.log_dir", "");
    declare_parameter<double>("rerunLog.max_log_file_mins", 5.0);
    declare_parameter<int>("rerunLog.img_interval", 10);

    declare_parameter<bool>("sound.enable", false);
    declare_parameter<string>("sound.sound_pack", "espeak");
    declare_parameter<bool>("sound.debug_logs", false);

    declare_parameter<string>("vision.image_topic", "/camera/camera/color/image_raw");
    declare_parameter<string>("vision.depth_image_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    declare_parameter<double>("vision.cam_pixel_width", 1280);
    declare_parameter<double>("vision.cam_pixel_height", 720);
    declare_parameter<double>("vision.cam_fov_x", 90);
    declare_parameter<double>("vision.cam_fov_y", 65);

    declare_parameter<string>("game_control_ip", "0.0.0.0");

    declare_parameter<string>("tree_file_path", "");
    declare_parameter<string>("vision_config_path", "");
    declare_parameter<string>("vision_config_local_path", "");

    declare_parameter<int>("recovery.retry_max_count", 2);
}

Brain::~Brain()
{

}

void Brain::init()
{

    config = std::make_shared<BrainConfig>();
    loadConfig();

    data = std::make_shared<BrainData>();
    locator = std::make_shared<Locator>();
    ballPredictor = std::make_shared<PosPredictor>(get_clock());
    log = std::make_shared<BrainLog>(this);
    tree = std::make_shared<BrainTree>(this);
    client = std::make_shared<RobotClient>(this);
    communication = std::make_shared<BrainCommunication>(this);


    locator->init(config->fieldDimensions, config->pfMinMarkerCnt, config->pfMaxResidual);


    tree->init();


    client->init();

    log->prepare();



    communication->initCommunication();

    data->lastSuccessfulLocalizeTime = get_clock()->now();
    data->timeLastDet = get_clock()->now();
    data->timeLastLineDet = get_clock()->now();
    data->timeLastGamecontrolMsg = get_clock()->now();
    data->ball.timePoint = get_clock()->now();


    auto now = get_clock()->now();
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        data->tmStatus[i].isAlive = false;
        data->tmStatus[i].timeLastCom = now;
    }
    data->tmLastCmdChangeTime = now;


    detectionsSubscription = create_subscription<vision_interface::msg::Detections>("/booster_vision/detection", SUB_STATE_QUEUE_SIZE, bind(&Brain::detectionsCallback, this, _1));
    subFieldLine = create_subscription<vision_interface::msg::LineSegments>("/booster_vision/line_segments", SUB_STATE_QUEUE_SIZE, bind(&Brain::fieldLineCallback, this, _1));
    odometerSubscription = create_subscription<booster_interface::msg::Odometer>("/odometer_state", SUB_STATE_QUEUE_SIZE, bind(&Brain::odometerCallback, this, _1));
    lowStateSubscription = create_subscription<booster_interface::msg::LowState>("/low_state", SUB_STATE_QUEUE_SIZE, bind(&Brain::lowStateCallback, this, _1));
    headPoseSubscription = create_subscription<geometry_msgs::msg::Pose>("/head_pose", SUB_STATE_QUEUE_SIZE, bind(&Brain::headPoseCallback, this, _1));
    recoveryStateSubscription = create_subscription<booster_interface::msg::RawBytesMsg>("fall_down_recovery_state", SUB_STATE_QUEUE_SIZE, bind(&Brain::recoveryStateCallback, this, _1));

    if (config->rerunLogEnableFile || config->rerunLogEnableTCP) {
        string imageTopic = get_parameter("vision.image_topic").as_string();
        imageSubscription = create_subscription<sensor_msgs::msg::Image>(imageTopic, SUB_STATE_QUEUE_SIZE, bind(&Brain::imageCallback, this, _1));
    }
    string depthTopic = get_parameter("vision.depth_image_topic").as_string();
    depthImageSubscription = create_subscription<sensor_msgs::msg::Image>(depthTopic, SUB_STATE_QUEUE_SIZE, bind(&Brain::depthImageCallback, this, _1));

    pubSoundPlay = create_publisher<std_msgs::msg::String>("/play_sound", 10);
    pubSpeak = create_publisher<std_msgs::msg::String>("/speak", 10);
    pubKickBall = create_publisher<brain::msg::Kick>("/kick_ball", 10);
}

void Brain::loadConfig()
{
    get_parameter("game.team_id", config->teamId);
    get_parameter("game.player_id", config->playerId);
    get_parameter("game.field_type", config->fieldType);
    get_parameter("game.player_role", config->playerRole);
    get_parameter("game.player_start_pos", config->playerStartPos);
    get_parameter("game.treat_person_as_robot", config->treatPersonAsRobot);
    get_parameter("game.number_of_players", config->numOfPlayers);

    get_parameter("robot.robot_height", config->robotHeight);
    get_parameter("robot.odom_factor", config->robotOdomFactor);
    get_parameter("robot.vx_factor", config->vxFactor);
    get_parameter("robot.yaw_offset", config->yawOffset);
    get_parameter("robot.vx_limit", config->vxLimit);
    get_parameter("robot.vy_limit", config->vyLimit);
    get_parameter("robot.vtheta_limit", config->vthetaLimit);

    get_parameter("strategy.ball_confidence_threshold", config->ballConfidenceThreshold);
    get_parameter("strategy.ball_confidence_decay_rate", config->ballConfidenceDecayRate);
    get_parameter("strategy.enable_stable_kick", config->enableStableKick);
    get_parameter("strategy.tm_ball_dist_threshold", config->tmBallDistThreshold);
    get_parameter("strategy.limit_near_ball_speed", config->limitNearBallSpeed);
    get_parameter("strategy.near_ball_speed_limit", config->nearBallSpeedLimit);
    get_parameter("strategy.near_ball_range", config->nearBallRange);

    get_parameter("obstacle_avoidance.collision_threshold", config->collisionThreshold);
    get_parameter("obstacle_avoidance.safe_distance", config->safeDistance);
    get_parameter("obstacle_avoidance.avoid_secs", config->avoidSecs);

    get_parameter("locator.min_marker_count", config->pfMinMarkerCnt);
    get_parameter("locator.max_residual", config->pfMaxResidual);

    get_parameter("enable_com", config->enableCom);

    // get_parameter("rerunLog.enable", config->rerunLogEnable);
    get_parameter("rerunLog.enable_tcp", config->rerunLogEnableTCP);
    get_parameter("rerunLog.server_ip", config->rerunLogServerIP);
    get_parameter("rerunLog.enable_file", config->rerunLogEnableFile);
    get_parameter("rerunLog.log_dir", config->rerunLogLogDir);
    get_parameter("rerunLog.max_log_file_mins", config->rerunLogMaxFileMins);
    get_parameter("rerunLog.img_interval", config->rerunLogImgInterval);

    get_parameter("sound.enable", config->soundEnable);
    get_parameter("sound.sound_pack", config->soundPack);
    get_parameter("sound.debug_logs", config->soundDebugLogs);

    get_parameter("tree_file_path", config->treeFilePath);

    get_parameter("vision.cam_pixel_width", config->camPixX);
    get_parameter("vision.cam_pixel_height", config->camPixY);
    double camDegX, camDegY;
    get_parameter("vision.cam_fov_x", camDegX);
    get_parameter("vision.cam_fov_y", camDegY);
    config->camAngleX = deg2rad(camDegX);
    config->camAngleY = deg2rad(camDegY);

    // 浠庤瑙?config 涓姞杞界浉鍏冲弬鏁?
    string visionConfigPath, visionConfigLocalPath;
    get_parameter("vision_config_path", visionConfigPath);
    get_parameter("vision_config_local_path", visionConfigLocalPath);
    if (!filesystem::exists(visionConfigPath)) {
        // 鎶ラ敊鐒跺悗閫€鍑?
        RCLCPP_ERROR(get_logger(), "vision_config_path %s not exists", visionConfigPath.c_str());
        exit(1);
    }
    // else
    YAML::Node vConfig = YAML::LoadFile(visionConfigPath);
    if (filesystem::exists(visionConfigLocalPath)) {
        YAML::Node vConfigLocal = YAML::LoadFile(visionConfigLocalPath);
        MergeYAML(vConfig, vConfigLocal);
    }
    config->camfx = vConfig["camera"]["intrin"]["fx"].as<double>();
    config->camfy = vConfig["camera"]["intrin"]["fy"].as<double>();
    config->camcx = vConfig["camera"]["intrin"]["cx"].as<double>();
    config->camcy = vConfig["camera"]["intrin"]["cy"].as<double>();

    auto extrin = vConfig["camera"]["extrin"];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            config->camToHead(i, j) = extrin[i][j].as<double>();
        }
    }
    prtDebug(format("camfx: %f, camfy: %f, camcx: %f, camcy: %f", config->camfx, config->camfy, config->camcx, config->camcy));
    string str_cam2head = "camToHead: \n";
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            str_cam2head += format("%.3f ", config->camToHead(i, j));
        }
        str_cam2head += "\n";
    }
    prtDebug(str_cam2head);


    config->handle();


    ostringstream oss;
    config->print(oss);
    prtDebug(oss.str());
}


void Brain::tick()
{
    // 杈撳嚭 debug & log 鐩稿叧淇℃伅
    // logObstacleDistance(); // 璁＄畻閲忓ぇ, 浠呴渶瑕佹椂浣跨敤
    logLags();
    logStatusToConsole();
    updateLogFile();

    updateMemory();
    updateBallPrediction();
    handleSpecialStates();
    handleCooperation();

    pubKickMsg();

    tree->tick();
    logDebugInfo();
    statusReport();
    playSoundForFun();
}

void Brain::pubKickMsg() {
    if (!pubKickBall) return;
    if (!data->ballDetected) return;
    brain::msg::Kick kickMsg;
    kickMsg.header.stamp = get_clock()->now();
    kickMsg.x = data->ball.posToRobot.x;
    kickMsg.y = data->ball.posToRobot.y;
    kickMsg.dir = toPInPI(data->kickDir - data->robotPoseToField.theta);

    double goal_x = config->fieldDimensions.length / 2;
    double goal_y = 0.0;
    Pose2D goalPose;
    goalPose.x = goal_x;
    goalPose.y = goal_y;
    double ball_x = data->ball.posToField.x;
    double ball_y = data->ball.posToField.y;
    double dist = std::sqrt((goal_x - ball_x) * (goal_x - ball_x) + (goal_y - ball_y) * (goal_y - ball_y));
    dist = std::abs(dist);
    double power = 0.0;

    if (dist > 6.0) {
        power = 2.0;
    } else {
        power = 6.0;
    }
    kickMsg.power = power;

    auto goalPose_r = data->field2robot(goalPose);
    kickMsg.goal_x = goalPose_r.x;
    kickMsg.goal_y = goalPose_r.y;

    kickMsg.robot_theta_to_field = data->robotPoseToField.theta;

    pubKickBall->publish(kickMsg);
}

void Brain::handleSpecialStates() {

    const double KICKOFF_DURATION = 10.0;
    string gameState = tree->getEntry<string>("gc_game_state");
    bool isKickoffSide = tree->getEntry<bool>("gc_is_kickoff_side");
    string gameSubStateType = tree->getEntry<string>("gc_game_sub_state_type");
    string gameSubState = tree->getEntry<string>("gc_game_sub_state");
    bool isFreekickKickoffSide = tree->getEntry<bool>("gc_is_sub_state_kickoff_side");
    auto now = get_clock()->now();

    if (gameState == "SET" && isKickoffSide) {
        data->isKickingOff = true;
        data->kickoffStartTime = now;
    } else if (msecsSince(data->kickoffStartTime) > KICKOFF_DURATION * 1000) {
        data->isKickingOff = false;
    }

    if (gameState == "PLAY" && gameSubStateType == "FREE_KICK" && isFreekickKickoffSide) {
        data->isFreekickKickingOff = true;
        data->freekickKickoffStartTime = now;
    } else if (msecsSince(data->freekickKickoffStartTime) > KICKOFF_DURATION * 1000) {
        data->isFreekickKickingOff = false;
        data->isDirectShoot = false;
    }

    static int lastScore = 0;
    if (data->score > lastScore) {
        tree->setEntry<bool>("we_just_scored", true);
        lastScore = data->score;
    }
    if (gameState == "SET") {
        tree->setEntry<bool>("we_just_scored", false);
    }
}

void Brain::handleCooperation() {
    auto log_ = [=](string msg) {
        log->setTimeNow();
        log->log("debug/handleCooperation", rerun::TextLog(msg));
    };
    log_("handle cooperation");

    const int COM_TIMEOUT = 5000;
    const double TM_BALL_TIMEOUT = 1000.0;
    const double CAPTAIN_STEAL_MARGIN = 0.30;

    int selfId = config->playerId;
    int selfIdx = selfId - 1;
    int numOfPlayers = config->numOfPlayers;
    data->tmMyCmd = 0;
    data->tmMyCmdId = 0;
    auto setCaptainAssignment = [&](int strikerId, int supporterId, const string &reason) {
        strikerId = max(0, strikerId);
        supporterId = max(0, supporterId);
        if (supporterId == strikerId) supporterId = 0;
        if (
            data->tmAssignedStrikerId != strikerId
            || data->tmAssignedSupporterId != supporterId
        ) {
            data->tmCaptainDecisionId += 1;
        }
        data->tmAssignedStrikerId = strikerId;
        data->tmAssignedSupporterId = supporterId;
        log_(format("captain assignment[%d]: striker=%d supporter=%d reason=%s",
            data->tmCaptainDecisionId,
            data->tmAssignedStrikerId,
            data->tmAssignedSupporterId,
            reason.c_str()));
    };
    auto adoptCaptainAssignment = [&](int captainId, int decisionId, int strikerId, int supporterId, const string &reason) {
        strikerId = max(0, strikerId);
        supporterId = max(0, supporterId);
        if (supporterId == strikerId) supporterId = 0;

        bool changed =
            data->tmCaptainDecisionId != decisionId
            || data->tmAssignedStrikerId != strikerId
            || data->tmAssignedSupporterId != supporterId;

        data->tmCaptainDecisionId = decisionId;
        data->tmAssignedStrikerId = strikerId;
        data->tmAssignedSupporterId = supporterId;

        if (changed) {
            log_(format("adopt captain assignment from P%d[%d]: striker=%d supporter=%d reason=%s",
                captainId,
                data->tmCaptainDecisionId,
                data->tmAssignedStrikerId,
                data->tmAssignedSupporterId,
                reason.c_str()));
        }
    };
    auto setMyTacticalRole = [&](int teamRole, bool isLead, const string &reason) {
        data->tmMyTeamRole = teamRole;
        data->tmImLead = isLead;
        tree->setEntry<bool>("is_lead", isLead);
        log_(reason);
    };

    vector<int> aliveTmIdxs = {};


    data->tmImAlive =
        (data->penalty[selfIdx] == PENALTY_NONE)
        && tree->getEntry<bool>("odom_calibrated");
    updateCostToKick();
    log_(format("ImAlive: %d, myCost: %.1f", data->tmImAlive, data->tmMyCost));


    int gcAliveCount = 0;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++)
    {
        if (data->penalty[i] == PENALTY_NONE) {
            gcAliveCount++;

            int tmId = i + 1;
            if (tmId == config->playerId) continue;

            auto tmStatus = data->tmStatus[i];
            log->setTimeNow();
            auto color = 0x00FFFFFF;
            if (!tmStatus.isAlive) color = 0x006666FF;
            else if (!tmStatus.isLead) color = 0x00CCCCFF;
            string label = format(
                "ID: %d, Cost: %.1f, State: %s, Role: %s, Down: %d, Cap: [%d] S=%d P=%d",
                tmId,
                tmStatus.cost,
                robotStateCodeName(tmStatus.robotState).c_str(),
                teamRoleCodeName(tmStatus.teamRole).c_str(),
                tmStatus.isFallen,
                tmStatus.captainDecisionId,
                tmStatus.assignedStrikerId,
                tmStatus.assignedSupporterId
            );
            log->logRobot(format("field/teammate-%d", tmId).c_str(), tmStatus.robotPoseToField, color, label);
            log->logBall(
            format("tm_ball-%d", tmId).c_str(),
            tmStatus.ballPosToField,
            tmStatus.ballDetected ? 0x00FFFFFF : (tmStatus.isAlive ? 0x006666FF : 0x003333FF),
            tmStatus.ballDetected,
            tmStatus.ballLocationKnown
            );
        }
    }
    log_(format("gcAliveCnt: %d", gcAliveCount));

    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue;

        if (
            data->penalty[i] != PENALTY_NONE
            || msecsSince(data->tmStatus[i].timeLastCom) > COM_TIMEOUT
        ) {
            data->tmStatus[i].isAlive = false;
            data->tmStatus[i].isLead = false;
        }

        if (data->tmStatus[i].isAlive) {
            aliveTmIdxs.push_back(i);
            log->setTimeNow();
            log->log(format("debug/tm_cost_scalar_%d", i + 1), rerun::Scalar(data->tmStatus[i].cost));
            log->log(format("debug/tm_lead_scalar_%d", i + 1), rerun::Scalar(data->tmStatus[i].isLead));
        }
    }
    log_(format("alive TM Count: %d", aliveTmIdxs.size()));

    // log 褰撳墠 alive 闃熷弸鐨勪俊鎭?
    log_(format("Self: cost: %.1f, isLead: %d", data->tmMyCost, data->tmImLead));


    static rclcpp::Time lastTmBallPosTime = get_clock()->now();
    const double RANGE_THRESHOLD = config->tmBallDistThreshold;
    int trustedTMIdx = -1;
    double minRange = 1e6;
    log_(format("Find ball info among %d alive TMs", aliveTmIdxs.size()));
    for (int i = 0; i < aliveTmIdxs.size(); i++) {
        auto status = data->tmStatus[aliveTmIdxs[i]];
        log_(format("TM %d, ballDetected: %d, ballRange: %.1f", i + 1, status.ballDetected, status.ballRange));
        if (status.ballDetected && status.ballRange < minRange) {
            log_(format("tm ball range(%.1f) < minRange(%.1f)", status.ballRange, minRange));
            double dist = norm(status.ballPosToField.x - data->robotPoseToField.x, status.ballPosToField.y - data->robotPoseToField.y);
            if (dist > RANGE_THRESHOLD) {
                log_(format("tm ball dist to me(%.1f) > threshold(%.1f), TM %d can be trusted", dist, RANGE_THRESHOLD, i+ 1));
                minRange = status.ballRange;
                trustedTMIdx = aliveTmIdxs[i];
            }  else {
                log_(format("tm ball dist to me(%.1f) < threshold(%.1f), TM %d can NOT be trusted", dist, RANGE_THRESHOLD, i+ 1));
            }
        }
    }
    if (trustedTMIdx >= 0) {
        log_(format("Reliable tm ball found. PlayerID = %d", trustedTMIdx + 1));
        data->tmBall.posToField = data->tmStatus[trustedTMIdx].ballPosToField;
        updateRelativePos(data->tmBall);

        tree->setEntry<bool>("tm_ball_pos_reliable", true);
        lastTmBallPosTime = get_clock()->now();
        if (!tree->getEntry<bool>("ball_location_known")) {
            log_("update ball.posToField");
            data->ball.posToField = data->tmBall.posToField;
            updateRelativePos(data->ball);
        }
    } else {
        log_("TM reported NO BALL or can not be trusted");
        if (msecsSince(lastTmBallPosTime) > TM_BALL_TIMEOUT) {
            log_("TM ball timeout reached");
            tree->setEntry<bool>("tm_ball_pos_reliable", false);
        }
    }

    bool switchRole;
    get_parameter("strategy.cooperation.enable_role_switch", switchRole);
    if (switchRole) {
        if (data->penalty[selfIdx] == PENALTY_NONE) {
            if (gcAliveCount < numOfPlayers) {
                log_("Not full team. I must be Striker");
                tree->setEntry<string>("player_role", "striker");
            }
        } else {
            if (gcAliveCount == numOfPlayers - 1) {
                log_("I am only on under penalty, I must be goal keeper");
                tree->setEntry<string>("player_role", "goal_keeper");
            }

        }
    }

    if (tree->getEntry<string>("gc_game_state") == "INITIAL") {
        tree->setEntry<string>("player_role", config->playerRole);
    }

    auto cmd = data->tmReceivedCmd;
    if (cmd != 0) {
        log_(format("received cmd %d from teammate", cmd));
        if (cmd == 100) {
            if (data->tmCaptainDecisionId == 0) {
                data->tmImLead = false;
                tree->setEntry<bool>("is_lead", false);
                log_("legacy teammate wants to take lead, i'll assist");
            } else {
                log_("ignore legacy lead cmd because captain assignment is active");
            }
        } else if (cmd > 10 && cmd < 20) {
            int newGoalieId = cmd - 10;
            log_("legacy goalie handoff received");
            if (newGoalieId == selfId) {
                tree->setEntry<string>("player_role", "goal_keeper");
                data->tmCaptainDecisionId = 0;
                data->tmAssignedStrikerId = 0;
                data->tmAssignedSupporterId = 0;
                log_("i become goalie");
                speak("i become goalie", true);
            } else {
                log_(format("teammate %d becomes goalie", newGoalieId));
            }
        } else {
            log_(format("unknown cmd %d from teammate", cmd));
        }

        data->tmReceivedCmd = 0;
    }

    string playerRole = tree->getEntry<string>("player_role");
    if (
        (tree->getEntry<string>("gc_game_state") == "READY" || tree->getEntry<string>("gc_game_sub_state") == "GET_READY")
        && gcAliveCount == numOfPlayers
    ) {
        tree->setEntry<string>("player_role", config->playerRole);
        if (config->playerRole == "goal_keeper") {
            data->tmMyTeamRole = TEAM_ROLE_GOALKEEPER;
            data->tmImLead = false;
        } else {
            data->tmMyTeamRole = TEAM_ROLE_STRIKER;
            data->tmImLead = true;
        }
        data->tmCaptainDecisionId = 0;
        data->tmAssignedStrikerId = 0;
        data->tmAssignedSupporterId = 0;
        tree->setEntry<bool>("is_lead", data->tmImLead);
        log_(format("all teammates on field. Back to initial role: %s", config->playerRole.c_str()));
        return;
    }


    double tmMinCost = 1e5;
    int myCostRank = 0;
    int myStrikerIDRank = 0;
    double BALL_CONTROL_COST_THRESHOLD = 3.0;
    get_parameter("strategy.cooperation.ball_control_cost_threshold", BALL_CONTROL_COST_THRESHOLD);

    vector<int> frontfieldIds = role_manager::collectFrontfieldIds(this, aliveTmIdxs, playerRole);
    auto isFrontfieldId = [&](int playerId) {
        return role_manager::isFrontfieldId(frontfieldIds, playerId);
    };
    for (int tmIdx : aliveTmIdxs) {
        auto tmStatus = data->tmStatus[tmIdx];
        if (tmStatus.role == "goal_keeper") continue;
        if (tmStatus.cost < tmMinCost) tmMinCost = tmStatus.cost;
        if (
            tmStatus.cost < data->tmMyCost - 1e-6
            || (fabs(tmStatus.cost - data->tmMyCost) < 1e-6 && tmIdx < selfIdx)
        ) {
            myCostRank++;
        }
        if (tmIdx < selfIdx) myStrikerIDRank++;
    }
    if (playerRole != "goal_keeper" && data->tmImAlive) {
        tmMinCost = min(tmMinCost, data->tmMyCost);
    }
    if (tmMinCost > 1e4) tmMinCost = data->tmMyCost;
    data->tmMyCostRank = myCostRank;
    data->myStrikerIDRank = myStrikerIDRank;

    int currentAssignedStriker = data->tmAssignedStrikerId;
    int currentAssignedSupporter = data->tmAssignedSupporterId;
    role_manager::CaptainAssignment currentAssignment{
        currentAssignedStriker,
        currentAssignedSupporter
    };
    bool currentStrikerStillValid = currentAssignedStriker > 0 && isFrontfieldId(currentAssignedStriker);
    bool currentSupporterStillValid =
        currentAssignedSupporter > 0
        && isFrontfieldId(currentAssignedSupporter)
        && currentAssignedSupporter != currentAssignedStriker;

    int remoteCaptainId = 0;
    int remoteCaptainDecisionId = 0;
    int remoteCaptainStrikerId = 0;
    int remoteCaptainSupporterId = 0;
    for (int tmIdx : aliveTmIdxs) {
        const auto &tmStatus = data->tmStatus[tmIdx];
        if (tmStatus.role != "goal_keeper" || !tmStatus.isAlive || tmStatus.isFallen) continue;
        if (tmStatus.captainDecisionId <= 0) continue;
        if (!isFrontfieldId(tmStatus.assignedStrikerId)) continue;

        int supporterId = tmStatus.assignedSupporterId;
        if (supporterId > 0 && (!isFrontfieldId(supporterId) || supporterId == tmStatus.assignedStrikerId)) {
            supporterId = 0;
        }

        int captainId = tmIdx + 1;
        if (
            tmStatus.captainDecisionId > remoteCaptainDecisionId
            || (
                tmStatus.captainDecisionId == remoteCaptainDecisionId
                && (remoteCaptainId == 0 || captainId < remoteCaptainId)
            )
        ) {
            remoteCaptainId = captainId;
            remoteCaptainDecisionId = tmStatus.captainDecisionId;
            remoteCaptainStrikerId = tmStatus.assignedStrikerId;
            remoteCaptainSupporterId = supporterId;
        }
    }

    if (playerRole == "goal_keeper" && data->tmImAlive) {
        const auto nextAssignment = role_manager::chooseCaptainAssignment(
            this,
            frontfieldIds,
            currentAssignment,
            CAPTAIN_STEAL_MARGIN
        );
        setCaptainAssignment(nextAssignment.strikerId, nextAssignment.supporterId, "goalkeeper_referee");
    } else if (remoteCaptainId > 0) {
        adoptCaptainAssignment(
            remoteCaptainId,
            remoteCaptainDecisionId,
            remoteCaptainStrikerId,
            remoteCaptainSupporterId,
            "remote_goalkeeper_referee"
        );
    } else if (
        data->tmCaptainDecisionId == 0
        || !currentStrikerStillValid
        || (currentAssignedSupporter > 0 && !currentSupporterStillValid)
    ) {
        int fallbackStrikerId = 0;
        if (playerRole != "goal_keeper" && data->tmImAlive) fallbackStrikerId = selfId;
        else if (!frontfieldIds.empty()) fallbackStrikerId = frontfieldIds.front();
        int fallbackSupporterId = 0;
        for (int playerId : frontfieldIds) {
            if (playerId == fallbackStrikerId) continue;
            fallbackSupporterId = playerId;
            break;
        }
        setCaptainAssignment(fallbackStrikerId, fallbackSupporterId, "fallback_local_default");
    }

    if (!data->tmImAlive) {
        setMyTacticalRole(TEAM_ROLE_UNKNOWN, false, "I am off field");
    } else if (playerRole == "goal_keeper") {
        setMyTacticalRole(TEAM_ROLE_GOALKEEPER, false, "I am goalkeeper captain");
    } else if (data->tmAssignedStrikerId > 0) {
        if (data->tmAssignedStrikerId == selfId) {
            setMyTacticalRole(TEAM_ROLE_STRIKER, true, "I obey captain assignment: striker");
        } else if (data->tmAssignedSupporterId == selfId) {
            setMyTacticalRole(TEAM_ROLE_SUPPORTER, false, "I obey captain assignment: supporter");
        } else {
            setMyTacticalRole(TEAM_ROLE_SUPPORTER, false, "I obey captain assignment: frontfield assist");
        }
    } else if (
        (tmMinCost < BALL_CONTROL_COST_THRESHOLD && data->tmMyCost > tmMinCost)
        || myCostRank >= 2
    ) {
        setMyTacticalRole(TEAM_ROLE_SUPPORTER, false, "I am not lead");
    } else {
        setMyTacticalRole(TEAM_ROLE_STRIKER, true, "I am lead");
    }

    string captainSource = "none";
    int captainSourceId = 0;
    if (playerRole == "goal_keeper" && data->tmImAlive) {
        captainSource = "self_goalkeeper";
        captainSourceId = selfId;
    } else if (remoteCaptainId > 0) {
        captainSource = "remote_goalkeeper";
        captainSourceId = remoteCaptainId;
    } else if (data->tmCaptainDecisionId > 0) {
        captainSource = "local_fallback";
        captainSourceId = selfId;
    }
    log_(format(
        "summary: role=%s teamRole=%s lead=%d captainSrc=%s[%d] captain[%d] S=%d P=%d tmMin=%.2f myCost=%.2f rank=%d idRank=%d",
        playerRole.c_str(),
        teamRoleCodeName(data->tmMyTeamRole).c_str(),
        data->tmImLead,
        captainSource.c_str(),
        captainSourceId,
        data->tmCaptainDecisionId,
        data->tmAssignedStrikerId,
        data->tmAssignedSupporterId,
        tmMinCost,
        data->tmMyCost,
        myCostRank,
        myStrikerIDRank
    ));

    return;
}

void Brain::updateMemory()
{
    updateBallMemory();
    updateRobotMemory();
    updateObstacleMemory();
    updateKickoffMemory();
}

void Brain::updateObstacleMemory() {

    auto obstacles = data->getObstacles();
    vector<GameObject> obs_new = {};

    const double OBS_EXPIRE_TIME = get_parameter("obstacle_avoidance.obstacle_memory_msecs").as_double();
    for (int i = 0; i < obstacles.size(); i++) {
        auto obs = obstacles[i];
        if (obs.label == "Ball") continue;
        if (msecsSince(obs.timePoint) > OBS_EXPIRE_TIME)  continue;


        updateRelativePos(obs);
        obs_new.push_back(obs);
    }


    if (
        (get_parameter("obstacle_avoidance.enable_freekick_avoid").as_bool() && isFreekickStartPlacing())
        || tree->getEntry<string>("gc_game_state") == "READY"
    ) {
        obs_new.push_back(data->ball);
    }

    data->setObstacles(obs_new);
    logObstacles();
}

void Brain::updateBallMemory()
{
    double secs = max(0.0, msecsSince(data->ball.timePoint) / 1000.0);

    double ballMemTimeout;
    get_parameter("strategy.ball_memory_timeout", ballMemTimeout);

    const double rawBallConfidence = max(0.0, data->ball.confidence);
    if (config->ballConfidenceDecayRate > 1e-6) {
        const double decayPerSec = 100.0 / config->ballConfidenceDecayRate;
        data->ballEffectiveConfidence = max(0.0, rawBallConfidence - secs * decayPerSec);
    } else {
        data->ballEffectiveConfidence = rawBallConfidence;
    }

    if (secs > ballMemTimeout) {
        data->ballEffectiveConfidence = 0.0;
    }

    const bool ballLocationKnown = data->ballEffectiveConfidence > 0.0;
    tree->setEntry<bool>("ball_location_known", ballLocationKnown);
    if (!ballLocationKnown) {
        tree->setEntry<bool>("ball_out", false);
    }

    updateRelativePos(data->ball);
    updateRelativePos(data->tmBall);
    tree->setEntry<double>("ball_range", data->ball.range);



    log->setTimeNow();
    log->logBall(
        "field/ball",
        data->ball.posToField,
        data->ballDetected ? 0x00FF00FF : 0x006600FF,
        data->ballDetected,
        ballLocationKnown
        );
    log->logBall(
        "field/tmBall",
        data->tmBall.posToField,
        0xFFFF00FF,
        tree->getEntry<bool>("tm_ball_pos_reliable"),
        tree->getEntry<bool>("tm_ball_pos_reliable")
        );
}

void Brain::updateRobotMemory() {
    auto robots = data->getRobots();
    vector<GameObject> newRobots = {};

    for (int i = 0; i < robots.size(); i++) {
        auto r = robots[i];


        if (msecsSince(r.timePoint) > 1000)  continue;


        updateRelativePos(r);
        newRobots.push_back(r);
    }

    data->setRobots(newRobots);

    logMemRobots();
}

void Brain::updateKickoffMemory() {

    static Point ballPos;
    const double BALL_MOVE_THRESHOLD_FACTOR = 0.15;
    const double BALL_MOVE_THRESHOLD_MIN = 0.3;
    auto ballMoved = [=]() {
        if (!data->ballDetected) return false;
        double range = data->ball.range;
        double threshold = max(range * BALL_MOVE_THRESHOLD_FACTOR, BALL_MOVE_THRESHOLD_MIN);
        double posChange = norm(data->ball.posToRobot.x - ballPos.x, data->ball.posToRobot.y - ballPos.y);
        return posChange > threshold;
    };
    static rclcpp::Time kickOffTime;
    const double TIMEOUT = 1000 * 10;
    auto timeReached = [=]() {
        return msecsSince(kickOffTime) > TIMEOUT;
    };
    bool isWaitingForKickoff = (
        (tree->getEntry<string>("gc_game_state") == "SET"  || tree->getEntry<string>("gc_game_state") == "READY")
        && !tree->getEntry<bool>("gc_is_kickoff_side")
    );
    bool isWaitingForFreekickKickoff = (
        (tree->getEntry<string>("gc_game_sub_state") == "SET" || tree->getEntry<string>("gc_game_sub_state") == "GET_READY")
        && !tree->getEntry<bool>("gc_is_sub_state_kickoff_side")
    );
    if ( isWaitingForFreekickKickoff || isWaitingForKickoff) {
        ballPos = data->ball.posToRobot;
        kickOffTime = get_clock()->now();
        tree->setEntry<bool>("wait_for_opponent_kickoff", true);
    } else if (tree->getEntry<bool>("wait_for_opponent_kickoff")) {
        if (ballMoved() || timeReached()) {
            tree->setEntry<bool>("wait_for_opponent_kickoff", false);
        }
    }
}

vector<double> Brain::getGoalPostAngles(const double margin)
{
    double leftX, leftY, rightX, rightY;

    leftX = config->fieldDimensions.length / 2;
    leftY = config->fieldDimensions.goalWidth / 2;
    rightX = config->fieldDimensions.length / 2;
    rightY = -config->fieldDimensions.goalWidth / 2;


    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++)
    {
        auto post = goalposts[i];
        if (post.name == "OL")
        {
            leftX = post.posToField.x;
            leftY = post.posToField.y;
        }
        else if (post.name == "OR")
        {
            rightX = post.posToField.x;
            rightY = post.posToField.y;
        }
    }

    const double theta_l = atan2(leftY - margin - data->ball.posToField.y, leftX - data->ball.posToField.x);
    const double theta_r = atan2(rightY + margin - data->ball.posToField.y, rightX - data->ball.posToField.x);

    vector<double> vec = {theta_l, theta_r};
    return vec;
}

double Brain::calcKickDir(double goalPostMargin) {
    double dir_rb_f = data->robotBallAngleToField;
    auto goalPostAngles = getGoalPostAngles(goalPostMargin);
    double theta_l = goalPostAngles[0];
    double theta_r = goalPostAngles[1];

    if (isAngleGood(goalPostMargin)) return dir_rb_f;

    double delta_l = fabs(toPInPI(theta_l - dir_rb_f));
    double delta_r = fabs(toPInPI(theta_r - dir_rb_f));
    if (delta_l < delta_r) return theta_l;
    // else
    return theta_r;
}

void Brain::updateCostToKick() {
    auto log_ = [=](string msg) {
        log->setTimeNow();
        log->log("debug/updateCostToKick", rerun::TextLog(msg));
    };
    double cost = 0.;


    // if (!data->ballDetected) cost += 2.0;
    double secsSinceBallDet = msecsSince(data->ball.timePoint) / 1000;
    cost += secsSinceBallDet;
    log_(format("ball not dectect cost: %.1f", secsSinceBallDet));


    if (!tree->getEntry<bool>("ball_location_known")) {
        cost += 5.0;
        log_(format("ball lost cost: %.1f", 5.0));
    }


    cost += data->ball.range;
    log_(format("ball range cost: %.1f", data->ball.range));



    if (distToObstacle(data->ball.yawToRobot) < 1.5) {
        log_(format("obstacle cost: %.1f", 2.0));
        cost += 0.5;
    }


    cost += fabs(data->ball.yawToRobot) / 1.0;
    log_(format("ball yaw cost: %.1f", fabs(data->ball.yawToRobot) / 1.0));



    int selfIdx = config->playerId - 1;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue;

        auto status = data->tmStatus[i];
        if (!status.isAlive) continue;

        double theta_tm2ball = atan2(status.ballPosToField.y - status.robotPoseToField.y, status.ballPosToField.x - status.robotPoseToField.x);
        double range_tm2ball = norm(status.ballPosToField.y - status.robotPoseToField.y, status.ballPosToField.x - status.robotPoseToField.x);
        double theta_me2ball = data->robotBallAngleToField;
        double range_me2ball = data->ball.range;
        double deltaTheta = fabs(toPInPI(theta_tm2ball - theta_me2ball));

        const double BUMP_DIST = 1.0;
        if (range_tm2ball < range_me2ball && sin(deltaTheta) * range_tm2ball < BUMP_DIST) {
            cost += 2.0;
            log_(format("bump cost: %.1f", 2.0));
        }
    }

    cost += fabs(toPInPI(data->kickDir - data->robotBallAngleToField)) * 0.4 / 0.3;
    log_(format("ajust cost: %.1f", fabs(toPInPI(data->kickDir - data->robotBallAngleToField)) * 0.4 / 0.3));


    if (data->recoveryState == RobotRecoveryState::HAS_FALLEN) {
        cost += 15.0;
        log_(format("fall cost: %.1f", 15.0));
    }


    if (!tree->getEntry<bool>("odom_calibrated")) {
        cost += 100;
        log_(format("localization cost: %.1f", 100.0));

    }

    double lastCost = data->tmMyCost;
    data->tmMyCost = lastCost * 0.8 + cost * 0.2;
    log_(format("lastCost: %.1f, newCost: %.1f, smoothCost: %.1f", lastCost, cost, data->tmMyCost));

    return;
}

bool Brain::isAngleGood(double goalPostMargin, string type) {
    double angle = 0;
    if (type == "kick") angle = data->robotBallAngleToField; // type=="kick" 鏈哄櫒浜哄埌鐞? field 鍧愭爣绯讳腑鐨勬柟鍚?
    if (type == "shoot") angle = data->robotPoseToField.theta; // type=="shoot" 鏈哄櫒浜烘湞鍚?


    auto goalPostAngles = getGoalPostAngles(goalPostMargin);
    double theta_l = goalPostAngles[0];
    double theta_r = goalPostAngles[1];

    if (theta_l - theta_r < M_PI / 3 * 2) {
        goalPostAngles = getGoalPostAngles(0.5);
        theta_l = goalPostAngles[0];
        theta_r = goalPostAngles[1];
    }

    return (theta_l > angle && theta_r < angle);
}

bool Brain::isPrimaryStriker() {
    string myRole = tree->getEntry<string>("player_role");
    if (myRole != "striker") return false;

    if (!config->enableCom) return true;

    // find first alive striker that is not me.
    auto firstAliveStrikerIdx = -1;
    auto myIdx = config->playerId - 1;

    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        auto status = data->tmStatus[i];
        if (data->penalty[i] == PENALTY_NONE && status.isAlive && status.role == "striker") {
            firstAliveStrikerIdx = i;
            break;
        }
    }

    if ( firstAliveStrikerIdx >= 0 && firstAliveStrikerIdx <  myIdx) return false; // 鏈?id 鏇村皬鐨勬椿鐨勫墠閿? 璁╀粬鏉ュ綋涓诲姏

    // else 娌℃湁 id 鏇村皬鐨勬椿鐨勫墠閿? 鎴戞槸涓诲姏
    return true;
}

bool Brain::isBallOut(double locCompareDist, double lineCompareDist)
{
    auto ball = data->ball;
    auto fd = config->fieldDimensions;

    if (fabs(ball.posToField.x) > fd.length / 2 + locCompareDist)
        return true;
    if (fabs(ball.posToField.y) > fd.width / 2 + locCompareDist)
        return true;

    auto fieldLines = data->getFieldLines();
    for (int i = 0; i < fieldLines.size(); i++) {
        auto line = fieldLines[i];
        if (
            (line.type == LineType::TouchLine || line.type == LineType::GoalLine)
            && line.confidence > 1.0
         ) {
            Point2D p = {ball.posToField.x, ball.posToField.y};
            // prtWarn(format("Ball: %.2f, %.2f PerpDist: %.2f", ball.posToField.x, ball.posToField.y, pointPerpDistToLine(p, line.posToField)));
            if (pointPerpDistToLine(p, line.posToField) > lineCompareDist) return true;
        }
    }
    return false;
}

void Brain::updateBallOut() {
    bool lastBallOut = tree->getEntry<bool>("ball_out");
    double range = lastBallOut ? 4.0 : 2.5;
    double threshold = config->ballOutThreshold;
    threshold += (data->isFreekickKickingOff ? 1.0 : 0.0); // 濡傛灉姝ｅ湪韪换鎰忕悆, 鍒欐斁瀹藉嚭鐣屽垽鏂?    threshold *= (lastBallOut ? 1.0 : 1.5); // 闃叉闇囪崱. 濡傛灉涓婃鍒ゆ柇涓哄嚭鐣? 鍒欐斁瀹藉嚭鐣屽垽鏂?    tree->setEntry<bool>("ball_out", isBallOut(threshold, 10.0) && data->ball.range < range); // 涓ユ牸閫氳繃瀹氫綅鍒ゆ柇鏄惁鍑虹晫
}

void Brain::updateBallPrediction()
{
    auto now = get_clock()->now();
    auto clearPredictionView = [this]() {
        log->setTimeNow();
        log->log("field/ball_prediction", rerun::Clear::FLAT);
        log->log("field/ball_breach_point", rerun::Clear::FLAT);
        log->log("field/ball_intercept_point", rerun::Clear::FLAT);
    };

    data->predictedBallPos.clear();
    data->ballPosPredictTime = now;
    data->ballWillBreach = false;
    data->ballBreachPoint = {data->ball.posToField.x, data->ball.posToField.y};
    data->ballBreachTime = now;
    data->ballInterceptPoint = {data->ball.posToField.x, data->ball.posToField.y};
    data->ballInterceptTime = now;
    tree->setEntry<bool>("ball_will_breach", false);
    log->setTimeNow();
    log->log("performance/ball_will_breach", rerun::Scalar(0));

    if (!ballPredictor || !tree->getEntry<bool>("ball_location_known")) {
        clearPredictionView();
        return;
    }

    double stepIntervalMsecs = 100.0;
    int stepCnt = 50;
    int linearSteps = 5;
    double linearRsqThreshold = 0.98;
    double acceleration = -0.4;
    get_parameter("ball_predictor.step_interval", stepIntervalMsecs);
    get_parameter("ball_predictor.step_cnt", stepCnt);
    get_parameter("ball_predictor.linear_steps", linearSteps);
    get_parameter("ball_predictor.linear_rsq_threshold", linearRsqThreshold);
    get_parameter("ball_predictor.acceleration", acceleration);

    const double maxSourceAgeMsecs = max(300.0, stepIntervalMsecs * 2.0);
    if (msecsSince(data->ball.timePoint) > maxSourceAgeMsecs) {
        clearPredictionView();
        log->setTimeNow();
        log->log("debug/ball_prediction", rerun::TextLog(format("skip: source too old age=%.0fms max=%.0fms", msecsSince(data->ball.timePoint), maxSourceAgeMsecs)));
        return;
    }

    vector<array<double, 2>> predictions;
    bool predicted = false;
    string predictSource = "filtered";
    string predictInfo;

    tie(predictions, predicted, predictInfo) = ballPredictor->predict_filtered(
        stepIntervalMsecs,
        stepCnt,
        acceleration,
        max(3, linearSteps)
    );

    if (!predicted || predictions.empty()) {
        predictSource = "linear";
        tie(predictions, predicted, predictInfo) = ballPredictor->predict_linear(
            stepIntervalMsecs,
            stepCnt,
            acceleration,
            max(3, linearSteps),
            linearRsqThreshold
        );
    }

    if (!predicted || predictions.empty()) {
        clearPredictionView();
        log->setTimeNow();
        log->log("debug/ball_prediction", rerun::TextLog(format("unavailable: source=%s info=%s", predictSource.c_str(), predictInfo.c_str())));
        return;
    }

    data->predictedBallPos = predictions;
    data->ballPosPredictTime = now;

    vector<rerun::Vec2D> predictionPoints;
    predictionPoints.reserve(predictions.size());
    for (const auto &prediction : predictions) {
        predictionPoints.push_back(rerun::Vec2D{prediction[0], -prediction[1]});
    }

    log->setTimeNow();
    log->log(
        "field/ball_prediction",
        rerun::Points2D(predictionPoints)
            .with_colors({0xFFA500FF})
            .with_radii({0.04})
    );

    const auto &fd = config->fieldDimensions;
    const double defendLineX = max(data->robotPoseToField.x, -fd.length / 2.0 + min(1.0, fd.goalAreaLength));
    const double defendHalfWidth = fd.penaltyAreaWidth / 2.0 + 0.3;
    const bool movingTowardSelfGoal = predictions.back()[0] < data->ball.posToField.x - 0.05;

    bool breachFound = false;
    for (size_t i = 1; i < predictions.size(); ++i) {
        const auto &p0 = predictions[i - 1];
        const auto &p1 = predictions[i];
        const double dx = p1[0] - p0[0];
        if (fabs(dx) < 1e-6) {
            continue;
        }
        const bool crossesDefendLine = (p0[0] - defendLineX) * (p1[0] - defendLineX) <= 0.0;
        if (!crossesDefendLine) {
            continue;
        }

        const double ratio = (defendLineX - p0[0]) / dx;
        if (ratio < 0.0 || ratio > 1.0) {
            continue;
        }

        const double breachY = p0[1] + (p1[1] - p0[1]) * ratio;
        if (!movingTowardSelfGoal || fabs(breachY) > defendHalfWidth) {
            continue;
        }

        data->ballWillBreach = true;
        data->ballBreachPoint = {defendLineX, breachY};
        data->ballInterceptPoint = data->ballBreachPoint;
        const double breachSecs = stepIntervalMsecs / 1000.0 * (static_cast<double>(i - 1) + ratio);
        data->ballBreachTime = data->ball.timePoint + rclcpp::Duration::from_seconds(breachSecs);
        data->ballInterceptTime = data->ballBreachTime;
        breachFound = true;
        break;
    }

    if (!breachFound) {
        double minRobotDist = 1e9;
        int closestIdx = -1;
        for (size_t i = 0; i < predictions.size(); ++i) {
            Pose2D predictedFieldPose{predictions[i][0], predictions[i][1], 0.0};
            auto predictedRobotPose = data->field2robot(predictedFieldPose);
            if (predictedRobotPose.x < -0.1 || predictedRobotPose.x > 2.0) {
                continue;
            }

            const double robotDist = norm(predictedRobotPose.x, predictedRobotPose.y);
            if (robotDist < minRobotDist) {
                minRobotDist = robotDist;
                closestIdx = static_cast<int>(i);
            }
        }

        if (movingTowardSelfGoal && closestIdx >= 0 && minRobotDist < 0.45) {
            data->ballWillBreach = true;
            data->ballBreachPoint = {predictions[closestIdx][0], predictions[closestIdx][1]};
            data->ballInterceptPoint = data->ballBreachPoint;
            const double breachSecs = stepIntervalMsecs / 1000.0 * static_cast<double>(closestIdx);
            data->ballBreachTime = data->ball.timePoint + rclcpp::Duration::from_seconds(breachSecs);
            data->ballInterceptTime = data->ballBreachTime;
            breachFound = true;
        }
    }

    tree->setEntry<bool>("ball_will_breach", data->ballWillBreach);
    log->setTimeNow();
    log->log("performance/ball_will_breach", rerun::Scalar(data->ballWillBreach ? 1.0 : 0.0));

    if (data->ballWillBreach) {
        log->log(
            "field/ball_breach_point",
            rerun::Points2D({{data->ballBreachPoint.x, -data->ballBreachPoint.y}})
                .with_colors({0xFF3333FF})
                .with_radii({0.08})
                .with_labels({format("t=%.2fs", max(0.0, (data->ballBreachTime - now).seconds()))})
        );
        log->log(
            "field/ball_intercept_point",
            rerun::Points2D({{data->ballInterceptPoint.x, -data->ballInterceptPoint.y}})
                .with_colors({0x33FF33FF})
                .with_radii({0.10})
                .with_labels({format("line=%.2f", defendLineX)})
        );
    } else {
        log->log("field/ball_breach_point", rerun::Clear::FLAT);
        log->log("field/ball_intercept_point", rerun::Clear::FLAT);
    }

    log->setTimeNow();
    log->log(
        "debug/ball_prediction",
        rerun::TextLog(format(
            "source=%s points=%d moving_to_self_goal=%d defend_line_x=%.2f will_breach=%d intercept=(%.2f, %.2f)",
            predictSource.c_str(),
            static_cast<int>(predictions.size()),
            movingTowardSelfGoal,
            defendLineX,
            data->ballWillBreach,
            data->ballInterceptPoint.x,
            data->ballInterceptPoint.y
        ))
    );
}

double Brain::distToBorder() {
    vector<Line> borders;
    auto fd = config->fieldDimensions;
    borders.push_back({fd.length / 2, fd.width / 2, -fd.length / 2, fd.width / 2});
    borders.push_back({fd.length / 2, -fd.width / 2, -fd.length / 2, -fd.width / 2});
    borders.push_back({fd.length / 2, fd.width / 2, fd.length / 2, -fd.width / 2});
    borders.push_back({-fd.length / 2, fd.width / 2, -fd.length / 2, -fd.width / 2});
    double maxDist = -100;
    Point2D robot = {data->robotPoseToField.x, data->robotPoseToField.y};
    for (int i = 0; i < borders.size(); i++) {
        auto line = borders[i];
        double dist = pointPerpDistToLine(robot, line);
        if (dist > maxDist) maxDist = dist;
    }
    return maxDist;
}

bool Brain::isBoundingBoxInCenter(BoundingBox boundingBox, double xRatio, double yRatio) {
    double x = (boundingBox.xmin + boundingBox.xmax) / 2.0;
    double y = (boundingBox.ymin + boundingBox.ymax) / 2.0;

    return (x  > config->camPixX * (1 - xRatio) / 2)
        && (x < config->camPixX * (1 + xRatio) / 2)
        && (y > config->camPixY * (1 - yRatio) / 2)
        && (y < config->camPixY * (1 + yRatio) / 2);
}

bool Brain::isDefensing() {
    bool isFreeKick = tree->getEntry<string>("gc_game_sub_state_type") == "FREE_KICK";
    bool isKickoffSide = tree->getEntry<bool>("gc_is_sub_state_kickoff_side");

    return isFreeKick && (!isKickoffSide);
}

void Brain::calibrateOdom(double x, double y, double theta)
{

    double x_or, y_or, theta_or; // or = odom to robot
    x_or = -cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    y_or = sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    theta_or = -data->robotPoseToOdom.theta;


    transCoord(x_or, y_or, theta_or,
               x, y, theta,
               data->odomToField.x, data->odomToField.y, data->odomToField.theta);


    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);


    double placeHolder;
    // ball
    transCoord(
        data->ball.posToRobot.x, data->ball.posToRobot.y, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        data->ball.posToField.x, data->ball.posToField.y, placeHolder
    );

    // robots
    auto robots = data->getRobots();
    for (int i = 0; i < robots.size(); i++) {
        updateFieldPos(robots[i]);
    }
    data->setRobots(robots);

    // goalposts
    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++) {
        updateFieldPos(goalposts[i]);
    }

    // markers
    auto markings = data->getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        updateFieldPos(markings[i]);
    }

    // relog
    log->setTimeNow();
    // logVisionBox(get_clock()->now());
    vector<GameObject> gameObjects = {};
    if(data->ballDetected) gameObjects.push_back(data->ball);
    for (int i = 0; i < markings.size(); i++) gameObjects.push_back(markings[i]);
    for (int i = 0; i < robots.size(); i++) gameObjects.push_back(robots[i]);
    for (int i = 0; i < goalposts.size(); i++) gameObjects.push_back(goalposts[i]);
    logDetection(gameObjects);
}

void Brain::playSound(string soundName, double blockMsecs, bool allowRepeat)
{
    if (!pubSoundPlay) return;

    static string _lastSound;
    static rclcpp::Time _lastTime;
    static double _lastBlockMsecs = 0;

    auto now = get_clock()->now();
    if (msecsSince(_lastTime) < _lastBlockMsecs) return;

    if (_lastSound == soundName && (!allowRepeat)) return;

    // else

    std_msgs::msg::String msg;
    msg.data = soundName;
    pubSoundPlay->publish(msg);

    _lastBlockMsecs = blockMsecs;
    _lastTime = now;
    _lastSound = soundName;
}

bool Brain::speak(string text, bool allowRepeat)
{
    auto log_ = [=](string msg) {
        if (!config->soundDebugLogs) return;
        log->setTimeNow();
        log->log("debug/speak", rerun::TextLog(format("status=%s text=%s", msg.c_str(), text.c_str())));
    };

    const double COOLDOWN_MSECS = 2000.;
    if (!pubSpeak) {
        log_("publisher not found");
        return false;
    }
    if (!config->soundEnable || config->soundPack != "espeak") {
        log_("config not compatible");
        return false;
    }

    static string _lastText;
    static rclcpp::Time _lastTime;

    if (msecsSince(_lastTime) < COOLDOWN_MSECS) {
        log_("cooldown in process");
        return false;
    }

    if (_lastText == text && (!allowRepeat)) {
        log_("repeat not allowed");
        return false;
    }

    // else
    _lastTime = get_clock()->now();
    std_msgs::msg::String msg;
    msg.data = text;
    pubSpeak->publish(msg);

    _lastText = text; return true;
}

double Brain::msecsSince(rclcpp::Time time)
{
    auto now = this->get_clock()->now();
    if (time.get_clock_type() != now.get_clock_type()) return 1e18;
    return (now - time).nanoseconds() / 1e6;
}

rclcpp::Time Brain::timePointFromHeader(std_msgs::msg::Header header) {
    auto stamp = header.stamp;
    // NOTE 浼间箮鏃犺 use_sim_time 鏄惁涓虹湡, 閮戒娇鐢ㄧ殑鏄?ROS_TIME
    auto sec = stamp.sec;
    auto nanosec = stamp.nanosec;
    if (sec <= 0 || nanosec <= 0) {
        sec = 1;
        nanosec = 1;
        prtErr(format("Negative time: sec: %d nanosec: %d"));
    }
    return rclcpp::Time(sec, nanosec, RCL_ROS_TIME); // should not crash
    // return rclcpp::Time(stamp.sec, stamp.nanosec, RCL_ROS_TIME);  // should crash sometimes
}

int Brain::getRobotStateCode()
{
    string decision = tree->getEntry<string>("decision");
    string playerRole = tree->getEntry<string>("player_role");
    string goalieMode = tree->getEntry<string>("goalie_mode");
    int controlState = tree->getEntry<int>("control_state");
    bool ballKnown = tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = tree->getEntry<bool>("tm_ball_pos_reliable");
    bool waitForOpponentKickoff = tree->getEntry<bool>("wait_for_opponent_kickoff");
    bool isUnderPenalty = tree->getEntry<bool>("gc_is_under_penalty");

    if (controlState == 0) return ROBOT_STATE_WAITING_START;
    if (controlState == 1) return ROBOT_STATE_MANUAL;
    if (controlState == 2) return ROBOT_STATE_ENTERING_FIELD;
    if (isUnderPenalty) return ROBOT_STATE_PENALIZED;
    if (waitForOpponentKickoff) return ROBOT_STATE_WAIT_OPPONENT_KICKOFF;
    if (decision == "intercept") return ROBOT_STATE_INTERCEPT;
    if (playerRole == "goal_keeper" && goalieMode == "guard") return ROBOT_STATE_GOALIE_GUARD;
    if (decision == "find") return ROBOT_STATE_FIND_BALL;
    if (decision == "chase") return ROBOT_STATE_CHASE_BALL;
    if (decision == "adjust") return ROBOT_STATE_ADJUST_BALL;
    if (decision == "kick" || decision == "safe_shoot") return ROBOT_STATE_KICK_BALL;
    if (decision == "cross") return ROBOT_STATE_CROSS_BALL;
    if (decision == "auto_visual_kick") return ROBOT_STATE_VISUAL_KICK;
    if (
        decision == "assist"
        && data->tmMyPassInitiator
        && data->tmMyPassOneTwoIntent
        && (
            data->tmMyOneTwoState == ONE_TWO_STATE_PASS_AND_GO
            || data->tmMyOneTwoState == ONE_TWO_STATE_ONE_TOUCH_RETURN
            || data->tmMyOneTwoState == ONE_TWO_STATE_WAIT_REACQUIRE
        )
    ) return ROBOT_STATE_ONE_TWO_GO;
    if (decision == "assist") return ROBOT_STATE_ASSIST;
    if (decision == "retreat") return ROBOT_STATE_RETREAT;
    if (ballKnown || tmBallPosReliable || data->ballDetected) return ROBOT_STATE_BALL_FOUND;
    return ROBOT_STATE_UNKNOWN;
}

string Brain::getRobotStateText()
{
    switch (getRobotStateCode()) {
    case ROBOT_STATE_WAITING_START: return string(u8"等待启动");
    case ROBOT_STATE_MANUAL: return string(u8"手动模式");
    case ROBOT_STATE_ENTERING_FIELD: return string(u8"入场定位中");
    case ROBOT_STATE_PENALIZED: return string(u8"罚下/重新入场中");
    case ROBOT_STATE_WAIT_OPPONENT_KICKOFF: return string(u8"等待对方开球");
    case ROBOT_STATE_GOALIE_GUARD: return string(u8"守门待命中");
    case ROBOT_STATE_FIND_BALL: return string(u8"正在找球");
    case ROBOT_STATE_BALL_FOUND: return string(u8"已找到球");
    case ROBOT_STATE_CHASE_BALL: return string(u8"正在追球");
    case ROBOT_STATE_ADJUST_BALL: return string(u8"已追到球，正在调整");
    case ROBOT_STATE_KICK_BALL: return string(u8"已追到球，正在踢球");
    case ROBOT_STATE_CROSS_BALL: return string(u8"已追到球，正在传球");
    case ROBOT_STATE_VISUAL_KICK: return string(u8"视觉踢球中");
    case ROBOT_STATE_ONE_TWO_GO: return string(u8"二过一前插中");
    case ROBOT_STATE_ASSIST: return string(u8"协防/辅助中");
    case ROBOT_STATE_RETREAT: return string(u8"回撤守门中");
    case ROBOT_STATE_INTERCEPT: return string(u8"守门拦截中");
    default: return string(u8"状态未知");
    }
}

string Brain::getRobotStateSpeech(int robotStateCode)
{
    switch (robotStateCode) {
    case ROBOT_STATE_WAITING_START: return "waiting to start";
    case ROBOT_STATE_MANUAL: return "manual mode";
    case ROBOT_STATE_ENTERING_FIELD: return "entering field and localizing";
    case ROBOT_STATE_PENALIZED: return "penalized and reentering field";
    case ROBOT_STATE_WAIT_OPPONENT_KICKOFF: return "waiting for opponent kickoff";
    case ROBOT_STATE_GOALIE_GUARD: return "goalkeeper guarding";
    case ROBOT_STATE_FIND_BALL: return "searching for the ball";
    case ROBOT_STATE_BALL_FOUND: return "ball found";
    case ROBOT_STATE_CHASE_BALL: return "chasing the ball";
    case ROBOT_STATE_ADJUST_BALL: return "ball reached, adjusting";
    case ROBOT_STATE_KICK_BALL: return "ball reached, kicking";
    case ROBOT_STATE_CROSS_BALL: return "ball reached, passing";
    case ROBOT_STATE_VISUAL_KICK: return "visual kick in progress";
    case ROBOT_STATE_ONE_TWO_GO: return "one-two go in progress";
    case ROBOT_STATE_ASSIST: return "assisting";
    case ROBOT_STATE_RETREAT: return "retreating to defend goal";
    case ROBOT_STATE_INTERCEPT: return "goalkeeper intercepting";
    default: return "state unknown";
    }
}


void Brain::joystickCallback(const booster_interface::msg::RemoteControllerState &joy)
{
    auto log_ = [=](string msg) {
        log->setTimeNow();
        log->log("debug/joystick", rerun::TextLog(msg));
    };
    // prtDebug("joy!!", RED_CODE);
    string soundPack = config->soundPack;

    // 閫氳繃鎵嬫焺鎺у埗鏈哄櫒浜? 涓嶉樆濉炴寜閿?
    if (
        fabs(joy.lx) > 0.1
        || fabs(joy.ly) > 0.1
        || fabs(joy.rx) > 0.1
        || fabs(joy.ry) > 0.1
    ) {
        tree->setEntry<bool>("go_manual", true);
        // prtWarn("GO Manual");
    } else {
        tree->setEntry<bool>("go_manual", false);
        // prtWarn("Axe manual take over end");
    }

    // 鎸夐敭鍝嶅簲椤哄簭: LT 缁勫悎閿? RT 缁勫悎閿? 鍗曢敭
    if (joy.lt && !joy.rt) { // LT 缁勫悎閿?
        // 鐢ㄤ簬鍦ㄧ嚎璋冭瘯鍙傛暟
        if (joy.hat_u || joy.hat_d)
        {
            config->vxFactor += 0.01 * (joy.hat_u ? 1.0 : -1.0);
            speak(format("vx factor: %.2f", config->vxFactor));
            prtDebug(
                format("vxFactor = %.2f  yawOffset = %.2f", config->vxFactor, config->yawOffset),
                RED_CODE
            );
        }

        if (joy.hat_l || joy.hat_r)
        {
            config->yawOffset += 0.01 * (joy.hat_r ? 1.0 : -1.0);
            speak(format("yaw offset: %.2f", config->yawOffset));
            prtDebug(
                format("vxFactor = %.2f  yawOffset = %.2f", config->vxFactor, config->yawOffset),
                RED_CODE
            );
        }

        // 鐢ㄤ簬鎺у埗鍒囨崲涓嶅悓鐨勭姸鎬?
        if (joy.x)
        {
            tree->setEntry<int>("control_state", 1);
            client->setVelocity(0., 0., 0.);
            client->moveHead(0., 0.);
            prtDebug("State => 1: CANCEL");
            // playSound("sad");
        }
        if (joy.a)
        {
            tree->setEntry<int>("control_state", 2);
            tree->setEntry<bool>("odom_calibrated", false);
            prtDebug("State => 2: RECALIBRATE");
            // playSound("search");
        }
        if (joy.b)
        {
            tree->setEntry<int>("control_state", 3);
            prtDebug("State => 3: ACTION");
            // playSound("exited");
        }
        else if (joy.y)
        {
            string curRole = tree->getEntry<string>("player_role");
            curRole == "striker" ? tree->setEntry<string>("player_role", "goal_keeper") : tree->setEntry<string>("player_role", "striker");
            prtDebug("SWITCH ROLE");
            log_("SWITCH ROLE");
            // playSound("talk");
        }
    }

    if (joy.rt) { // RT 缁勫悎閿?
        // Nothing for now
    }

    // else, 鍗曢敭浣?
    if (!joy.lt && !joy.rt) {
        if (joy.lb) {
            tree->setEntry<bool>("assist_chase", true);
            prtDebug("Assit Chase");
            playSound(soundPack + "-chase", 5000);
        } else {
            tree->setEntry<bool>("assist_chase", false);
            // prtWarn("Exit Assit Chase");
        }
        if (joy.rb) {
            tree->setEntry<bool>("assist_kick", true);
            prtDebug("Assit Kick");
            playSound(soundPack + "-kick", 5000);
        } else {
            tree->setEntry<bool>("assist_kick", false);
            // prtWarn("Exit Assit Kick");
        }

        if (joy.hat_u) {
            playSound(soundPack + "-celebrate", 2000, true);
        } else if (joy.hat_d) {
            playSound(soundPack + "-regret", 2000, true);
        } else if (joy.hat_r) {
            playSound(soundPack + "-ready", 2000, true);
        } else if (joy.hat_l) {
            playSound(soundPack + "-taunt", 2000, true);
        }
    }
}

void Brain::gameControlCallback(const game_controller_interface::msg::GameControlData &msg)
{
    data->timeLastGamecontrolMsg = get_clock()->now();

    // 澶勭悊姣旇禌鐨勪竴绾х姸鎬?
    auto lastGameState = tree->getEntry<string>("gc_game_state"); // 姣旇禌鐨勪竴绾х姸鎬?
    vector<string> gameStateMap = {
        "INITIAL", // 鍒濆鍖栫姸鎬? 鐞冨憳鍦ㄥ満澶栧噯澶?
        "READY",   // 鍑嗗鐘舵€? 鐞冨憳杩涘満, 骞惰蛋鍒拌嚜宸辩殑濮嬪彂浣嶇疆
        "SET",     // 鍋滄鍔ㄤ綔, 绛夊緟瑁佸垽鏈哄彂鍑哄紑濮嬫瘮璧涚殑鎸囦护
        "PLAY",    // 姝ｅ父姣旇禌
        "END"      // 姣旇禌缁撴潫
    };
    string gameState = gameStateMap[static_cast<int>(msg.state)];
    tree->setEntry<string>("gc_game_state", gameState);
    bool isKickOffSide = (msg.kick_off_team == config->teamId); // 鎴戞柟鏄惁鏄紑鐞冩柟
    tree->setEntry<bool>("gc_is_kickoff_side", isKickOffSide);

    // 澶勭悊姣旇禌鐨勪簩绾х姸鎬?
    string gameSubStateType;
    switch (static_cast<int>(msg.secondary_state)) {
        case 0:
            gameSubStateType = "NONE";
            data->realGameSubState = "NONE";
            break;
        case 3:
            gameSubStateType = "TIMEOUT"; // 鍖呭惈涓ら槦 timeout 鍜?瑁佸垽 timeout
            data->realGameSubState = "TIMEOUT";
            break;
        // 鏆傛椂涓嶅鐞嗗叾瀹冪姸鎬? 闄?TIMEOUT 澶? 閮芥寜 FREE_KICK 澶勭悊
        case 4:
            // gameSubStateType = "DIRECT_FREEKICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "DIRECT_FREEKICK";
            data->isDirectShoot = true;
            break;
        case 5:
            // gameSubStateType = "INDIRECT_FREEKICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "INDIRECT_FREEKICK";
            break;
        case 6:
            // gameSubStateType = "PENALTY_KICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "PENALTY_KICK";
            data->isDirectShoot = true;
            break;
        case 7:
            // gameSubStateType = "CORNER_KICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "CORNER_KICK";
            break;
        case 8:
            // gameSubStateType = "GOAL_KICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "GOAL_KICK";
            data->isDirectShoot = true;
            break;
        case 9:
            // gameSubStateType = "THROW_IN";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "THROW_IN";
            break;
        default:
            gameSubStateType = "FREE_KICK";
            break;
    }
    vector<string> gameSubStateMap = {"STOP", "GET_READY", "SET"};                               // STOP: 鍋滀笅鏉? -> GET_READY: 绉诲姩鍒拌繘鏀绘垨闃插畧浣嶇疆; -> SET: 绔欎綇涓嶅姩
    string gameSubState = gameSubStateMap[static_cast<int>(msg.secondary_state_info[1])];
    tree->setEntry<string>("gc_game_sub_state_type", gameSubStateType);
    tree->setEntry<string>("gc_game_sub_state", gameSubState);
    bool isSubStateKickOffSide = (static_cast<int>(msg.secondary_state_info[0]) == config->teamId); // 鍦ㄤ簩绾х姸鎬佷笅, 鎴戞柟鏄惁鏄紑鐞冩柟. 渚嬪, 褰撳墠浜岀骇鐘舵€佷负浠绘剰鐞? 鎴戞柟鏄惁鏄紑浠绘剰鐞冪殑涓€鏂?
    tree->setEntry<bool>("gc_is_sub_state_kickoff_side", isSubStateKickOffSide);

    // cout << "game state: " << gameState << " game sub state type: " << gameSubStateType << endl;
    // 鎵惧埌闃熺殑淇℃伅
    game_controller_interface::msg::TeamInfo myTeamInfo;
    game_controller_interface::msg::TeamInfo oppoTeamInfo;
    if (msg.teams[0].team_number == config->teamId)
    {
        myTeamInfo = msg.teams[0];
        oppoTeamInfo = msg.teams[1];
    }
    else if (msg.teams[1].team_number == config->teamId)
    {
        myTeamInfo = msg.teams[1];
        oppoTeamInfo = msg.teams[0];
    }
    else
    {
        // 鏁版嵁鍖呬腑娌℃湁鍖呭惈鎴戜滑鐨勯槦锛屼笉搴旇鍐嶅鐞嗕簡
        prtErr(format("received invalid game controller message team0 %d, team1 %d, teamId %d",
            msg.teams[0].team_number, msg.teams[1].team_number, config->teamId));
        return;
    }

    int liveCount = 0;
    int oppoLiveCount = 0;
    // 澶勭悊鍒ょ綒鐘舵€? penalty[playerId - 1] 浠ｈ〃鎴戞柟鐨勭悆鍛樻槸鍚﹀浜庡垽缃氱姸鎬? 澶勭悊鍒ょ綒鐘舵€佹剰鍛崇潃涓嶈兘绉诲姩
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        data->penalty[i] = static_cast<int>(myTeamInfo.players[i].penalty);

        if (static_cast<int>(myTeamInfo.players[i].red_card_count) > 0) {
            data->penalty[i] = PENALTY_SUBSTITUTE;
        }

        if (data->penalty[i] == PENALTY_NONE) liveCount++;
        data->oppoPenalty[i] = static_cast<int>(oppoTeamInfo.players[i].penalty);

        if (static_cast<int>(oppoTeamInfo.players[i].red_card_count) > 0) {
            data->oppoPenalty[i] = PENALTY_SUBSTITUTE;
        }

        if (data->oppoPenalty[i] == PENALTY_NONE) oppoLiveCount++;
    }
    data->liveCount = liveCount;
    data->oppoLiveCount = oppoLiveCount;

    // cout << "penalty: " << data->penalty[0] << " " << data->penalty[1] << " " << data->penalty[2] << " " << data->penalty[3] << endl;
    // cout << "oppo penalty: " << data->oppoPenalty[0] << " " << data->oppoPenalty[1] << " " << data->oppoPenalty[2] << " " << data->oppoPenalty[3] << endl;
    bool lastIsUnderPenalty = tree->getEntry<bool>("gc_is_under_penalty");
    bool isUnderPenalty = (data->penalty[config->playerId - 1] != PENALTY_NONE); // 褰撳墠 robot 鏄惁琚垽缃氫腑
    tree->setEntry<bool>("gc_is_under_penalty", isUnderPenalty);
    if (isUnderPenalty && !lastIsUnderPenalty) tree->setEntry<bool>("odom_calibrated", false); // 琚垽缃氫簡, 鍒欓渶瑕侀噸鏂拌繘鍦? 鍥犳闇€瑕侀噸鏂板畾浣?

    // log game state
    log->setTimeNow();
    log->logToScreen(
        "tick/gamecontrol",
        format("Player: %d  Role: %s PrimaryStriker: %s GameState: %s  SubStateType: %s  SubState: %s UnderPenalty: %d isKickoff: %d isSubStateKickoff: %d",
            config->playerId, tree->getEntry<string>("player_role").c_str(), isPrimaryStriker() ? "Yes" : "No", gameState.c_str(), gameSubStateType.c_str(), gameSubState.c_str(), isUnderPenalty, isKickOffSide, isSubStateKickOffSide
            ),
        0xFFFFFFFF,
        30.0
    );

    // FOR FUN 澶勭悊杩涚悆鍚庣殑搴嗙鎸ユ墜鐨勯€昏緫
    data->score = static_cast<int>(myTeamInfo.score);
    data->oppoScore = static_cast<int>(oppoTeamInfo.score);
}

void Brain::detectionsCallback(const vision_interface::msg::Detections &msg)
{
    // std::lock_guard<std::mutex> guard(data->brainMutex);

    // auto detection_time_stamp = msg.header.stamp;
    // rclcpp::Time timePoint(detection_time_stamp.sec, detection_time_stamp.nanosec);
    data->camConnected = true;
    auto timePoint = timePointFromHeader(msg.header);

    auto now = get_clock()->now();
    data->timeLastDet = timePoint; // 鐢ㄤ簬鍦ㄨ皟璇曚腑杈撳嚭寤惰繜淇℃伅

    auto gameObjects = getGameObjects(msg);

    // 瀵规娴嬪埌鐨勫璞¤繘琛屽垎缁?
    vector<GameObject> balls, goalposts, persons, robots, obstacles, markings;
    for (int i = 0; i < gameObjects.size(); i++)
    {
        const auto &obj = gameObjects[i];
        if (obj.label == "Ball")
            balls.push_back(obj);
        if (obj.label == "Goalpost")
            goalposts.push_back(obj);
        if (obj.label == "Person")
        {
            persons.push_back(obj);

            // 涓轰簡璋冭瘯鏂逛究, 鍙互鍦?config 涓缃?treat_person_as_robot, 浣垮緱 Person 琚綋浣?Robot 澶勭悊
            if (config->treatPersonAsRobot)
                robots.push_back(obj);
        }
        if (obj.label == "Opponent")
            robots.push_back(obj);
        if (obj.label == "LCross" || obj.label == "TCross" || obj.label == "XCross" || obj.label == "PenaltyPoint")
            markings.push_back(obj);
    }

    // 鍒嗗埆澶勭悊鍒嗙粍鍚庣殑瀵硅薄
    detectProcessBalls(balls);
    detectProcessGoalposts(goalposts);
    detectProcessMarkings(markings);
    detectProcessRobots(robots);

    // 澶勭悊骞惰褰曡閲庝俊鎭?
    detectProcessVisionBox(msg);

    // logVisionBox(timePoint);
    logDetection(gameObjects);
}

void Brain::updateLinePosToField(FieldLine& line) {
    double __; // __ is a placeholder for transformations
    transCoord(
        line.posToRobot.x0, line.posToRobot.y0, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        line.posToField.x0, line.posToField.y0, __
    );
    transCoord(
        line.posToRobot.x1, line.posToRobot.y1, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        line.posToField.x1, line.posToField.y1, __
    );
}

void Brain::fieldLineCallback(const vision_interface::msg::LineSegments &msg)
{
    // auto timestamp = msg.header.stamp;
    // rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);
    auto timePoint = timePointFromHeader(msg.header);

    auto now = get_clock()->now();
    data->timeLastLineDet = timePoint; // 鐢ㄤ簬鍦ㄨ皟璇曚腑杈撳嚭寤惰繜淇℃伅

    vector<FieldLine> lines = {};
    FieldLine line;

    double x0, y0, x1, y1, __; // __ is a placeholder for transformations
    for (int i = 0; i < msg.coordinates.size() / 4; i++) {
        int index = i * 4;
        line.posToRobot.x0 = msg.coordinates[index]; line.posOnCam.x0 = msg.coordinates_uv[index];
        line.posToRobot.y0 = msg.coordinates[index + 1]; line.posOnCam.y0 = msg.coordinates_uv[index + 1];
        line.posToRobot.x1 = msg.coordinates[index + 2]; line.posOnCam.x1 = msg.coordinates_uv[index + 2];
        line.posToRobot.y1 = msg.coordinates[index + 3]; line.posOnCam.y1 = msg.coordinates_uv[index + 3];
        updateLinePosToField(line);
        line.timePoint = timePoint;
        // TODO infer line dir and id

        lines.push_back(line);
    }
    lines = processFieldLines(lines);
    data->setFieldLines(lines);

    { // log processed lines
        log->setTimeSeconds(timePoint.seconds());
        vector<rerun::LineStrip2D> logLinesOnField = {};
        vector<rerun::LineStrip2D> logLinesOnCam = {};
        vector<rerun::LineStrip2D> logLinesOnRobotFrame = {};
        vector<string> logLabels = {};
        vector<unsigned int> logColors = {};

        for (int i = 0; i < lines.size(); i++) {
            auto line = lines[i];
            logLinesOnRobotFrame.push_back(rerun::LineStrip2D({
                {line.posToRobot.x0, -line.posToRobot.y0},
                {line.posToRobot.x1, -line.posToRobot.y1},
            }));
            logLinesOnField.push_back(rerun::LineStrip2D({
                {line.posToField.x0, -line.posToField.y0},
                {line.posToField.x1, -line.posToField.y1},
            }));
            logLinesOnCam.push_back(rerun::LineStrip2D({
                {line.posOnCam.x0, line.posOnCam.y0},
                {line.posOnCam.x1, line.posOnCam.y1},
            }));
            string label;
            unsigned int color = 0xFFFF00FF;
            if (line.type == LineType::GoalLine) {
                label = "GoalLine";
                color = 0xFF0000FF;
            }
            else if (line.type == LineType::TouchLine) {
                label = "TouchLine";
                color = 0xFF0000FF;
            }
            else if (line.type == LineType::MiddleLine) label = "MiddleLine";
            else if (line.type == LineType::PenaltyArea) label = "PenaltyArea";
            else if (line.type == LineType::GoalArea) label = "GoalArea";
            else if (line.type == LineType::NA) label = "NA";
            else label = "Other"; // should not see this label logged

            label += format(" c = %.1f", line.confidence);

            // if (line.dir == LineDir::Horizontal) label += " Horizontal";
            // else if (line.dir == LineDir::Vertical) label += " Vertical";
            // else label += " NA";

            logLabels.push_back(label);
            logColors.push_back(color);
        };
        log->log(
            "field/det_lines",
            rerun::LineStrips2D(logLinesOnField)
                .with_colors(logColors)
                .with_radii(0.04)
                .with_draw_order(20)
                .with_labels(logLabels)
        );
        // log->log(
        //     "robotframe/det_lines",
        //     rerun::LineStrips2D(logLinesOnRobotFrame)
        //         .with_colors(logColors)
        //         .with_radii(0.04)
        //         .with_draw_order(20)
        //         .with_labels(logLabels)
        // );
        log->log(
            "image/det_lines",
            rerun::LineStrips2D(logLinesOnCam)
                .with_colors(logColors)
                .with_radii(1.0)
                .with_draw_order(20)
                .with_labels(logLabels)
        );
    }

    { // log original lines
        log->setTimeSeconds(timePoint.seconds());
        vector<rerun::LineStrip2D> logLinesOnField = {};
        vector<rerun::LineStrip2D> logLinesOnCam = {};
        vector<rerun::LineStrip2D> logLinesOnRobotFrame = {};

        for (int i = 0; i < lines.size(); i++) {
            auto line = lines[i];
            logLinesOnRobotFrame.push_back(rerun::LineStrip2D({
                {line.posToRobot.x0, -line.posToRobot.y0},
                {line.posToRobot.x1, -line.posToRobot.y1},
            }));
            logLinesOnField.push_back(rerun::LineStrip2D({
                {line.posToField.x0, -line.posToField.y0},
                {line.posToField.x1, -line.posToField.y1},
            }));
            logLinesOnCam.push_back(rerun::LineStrip2D({
                {line.posOnCam.x0, line.posOnCam.y0},
                {line.posOnCam.x1, line.posOnCam.y1},
            }));
        };
        // log->log(
        //     "field/det_lines_raw",
        //     rerun::LineStrips2D(logLinesOnField)
        //         .with_colors(0xCCCCFFCC)
        //         .with_radii(0.1)
        //         .with_draw_order(10)
        // );
        // log->log(
        //     "robotframe/det_lines_raw",
        //     rerun::LineStrips2D(logLinesOnRobotFrame)
        //         .with_colors(0xCCCCFFCC)
        //         .with_radii(0.1)
        //         .with_draw_order(10)
        // );
        // log->log(
        //     "image/det_lines_raw",
        //     rerun::LineStrips2D(logLinesOnCam)
        //         .with_colors(0xCCCCFFCC)
        //         .with_radii(3.0)
        //         .with_draw_order(10)
        // );
    }
}

void Brain::odometerCallback(const booster_interface::msg::Odometer &msg)
{

    data->robotPoseToOdom.x = msg.x * config->robotOdomFactor;
    data->robotPoseToOdom.y = msg.y * config->robotOdomFactor;
    data->robotPoseToOdom.theta = msg.theta;

    // 鏍规嵁 Odom 淇℃伅, 鏇存柊鏈哄櫒浜哄湪 Field 鍧愭爣绯讳腑鐨勪綅缃?
    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);

    // 鍙戝竷tf鍙樻崲
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";

    // 璁剧疆骞崇Щ閮ㄥ垎
    transform.transform.translation.x = data->robotPoseToOdom.x;
    transform.transform.translation.y = data->robotPoseToOdom.y;
    transform.transform.translation.z = 0.0;

    // 璁剧疆鏃嬭浆閮ㄥ垎锛堜粠娆ф媺瑙掕浆鎹负鍥涘厓鏁帮級
    tf2::Quaternion q;
    q.setRPY(0, 0, data->robotPoseToOdom.theta);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    log->setTimeNow();
    log->log("debug/odom_callback", rerun::TextLog(format("x: %.1f, y: %.1f, z: %.1f", data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta)));

    // 骞挎挱tf鍙樻崲
    tf_broadcaster_->sendTransform(transform);

    // Log Odom 淇℃伅

    log->setTimeNow();
    auto color = 0x00FF00FF;
    if (!data->tmImAlive) color = 0x006600FF;
    else if (!data->tmImLead) color = 0x00CC00FF;
    string label = format("Cost: %.1f", data->tmMyCost);
    log->logRobot("field/robot", data->robotPoseToField, color, label, true);
}

void Brain::lowStateCallback(const booster_interface::msg::LowState &msg)
{
    data->headYaw = msg.motor_state_serial[0].q;
    data->headPitch = msg.motor_state_serial[1].q;
    log->log("debug/head_angles", rerun::TextLog(format("pitch: %.1f, yaw: %.1f", data->headYaw, data->headPitch)));
}

void Brain::imageCallback(const sensor_msgs::msg::Image &msg)
{

    static int counter = 0;
    counter++;
    if (counter % config->rerunLogImgInterval == 0)
    {
        // 鏈槻姝㈡憚鍍忓ご杩炴帴涓嶅ソ鏃? 鑷姩闄嶄綆鍒嗚鲸鐜? 褰卞搷 CamTrackBall 鐨勮绠? 鏇存柊鍒嗚鲸鐜囬厤缃?
        config->camPixX = msg.width;
        config->camPixY = msg.height;
        log->log("debug/imageCallback", rerun::TextLog(format("img width: %.d, img height: %.d", msg.width, msg.height)));

        cv::Mat image;
        // 鏍规嵁鍥惧儚缂栫爜鏍煎紡杩涜澶勭悊
        if (msg.encoding == "nv12" || msg.encoding == "NV12") {
            // NV12: Y plane (H x W) + interleaved UV (H/2 x W)
            size_t expected = (size_t)(msg.width * msg.height * 3 / 2);
            if (msg.data.size() < expected) {
                prtErr(format("NV12 buffer too small. got %zu expect >= %zu", msg.data.size(), expected));
                return;
            }
            cv::Mat yuv(msg.height + msg.height / 2, msg.width, CV_8UC1, const_cast<uint8_t*>(msg.data.data()));
            cv::cvtColor(yuv, image, cv::COLOR_YUV2BGR_NV12);
        } else if (msg.encoding == "bgra8") {
            // 鍒涘缓 OpenCV Mat 瀵硅薄锛屽鐞?BGRA 鏍煎紡鍥惧儚
            image = cv::Mat(msg.height, msg.width, CV_8UC4, const_cast<uint8_t *>(msg.data.data()));
            cv::Mat imageBGR;
            // 灏?BGRA 杞崲涓?BGR锛屽拷鐣?Alpha 閫氶亾
            cv::cvtColor(image, imageBGR, cv::COLOR_BGRA2BGR);
            image = imageBGR;
        } else if (msg.encoding == "bgr8") {
            // 鍘熸湁 BGR8 澶勭悊閫昏緫
            image = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
        } else if (msg.encoding == "rgb8") {
            // 鍘熸湁 RGB8 澶勭悊閫昏緫
            image = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        } else {
            // 澶勭悊鍏朵粬缂栫爜鏍煎紡锛屾垨鑰呰褰曢敊璇棩蹇?
            prtErr(format("Unsupported image encoding: %s", msg.encoding.c_str()));
            return;
        }

        // 鍘嬬缉鍥惧儚
        std::vector<uint8_t> compressed_image;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 10}; // 10 琛ㄧず鍘嬬缉璐ㄩ噺锛屽彲浠ユ牴鎹渶瑕佽皟鏁?
        cv::imencode(".jpg", image, compressed_image, compression_params);

        // 灏嗗帇缂╁悗鐨勫浘鍍忔暟鎹紶閫掔粰 rerun
        // double time = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
        // log->setTimeSeconds(time);
        log->setTimeSeconds(timePointFromHeader(msg.header).seconds());
        log->log("image/img", rerun::EncodedImage::from_bytes(compressed_image));
    }
}

void Brain::headPoseCallback(const geometry_msgs::msg::Pose& msg)
{
    // 璁＄畻 head_to_base 鐭╅樀
    Eigen::Matrix4d headToBase = Eigen::Matrix4d::Identity();

    // 浠庡洓鍏冩暟鑾峰彇鏃嬭浆鐭╅樀
    Eigen::Quaterniond q(
        msg.orientation.w,
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z
    );
    headToBase.block<3,3>(0,0) = q.toRotationMatrix();

    // 璁剧疆骞崇Щ鍚戦噺
    headToBase.block<3,1>(0,3) = Eigen::Vector3d(
        msg.position.x,
        msg.position.y,
        msg.position.z
    );

    // // 瀹氫箟骞惰绠?cam_to_head 鐭╅樀
    // Eigen::Matrix4d camToHead;
    // camToHead << 0,  0,  1,  0,
    //             -1,  0,  0,  0,
    //              0, -1,  0,  0,
    //              0,  0,  0,  1;

    // 璁＄畻 cam_to_base 鐭╅樀骞跺瓨鍌?
    data->camToRobot = headToBase * config->camToHead;
}

void Brain::recoveryStateCallback(const booster_interface::msg::RawBytesMsg &msg)
{
    // uint8_t state; // IS_READY = 0, IS_FALLING = 1, HAS_FALLEN = 2, IS_GETTING_UP = 3,
    // uint8_t is_recovery_available; // 1 for available, 0 for not available
    // 浣跨敤 RobotRecoveryState 缁撴瀯锛屽皢msg閲岄潰鐨刴sg杞崲涓篟obotRecoveryState
    try
    {
        const std::vector<unsigned char>& buffer = msg.msg;
        RobotRecoveryStateData recoveryState;
        memcpy(&recoveryState, buffer.data(), buffer.size());

        vector<RobotRecoveryState> recoveryStateMap = {
            RobotRecoveryState::IS_READY,
            RobotRecoveryState::IS_FALLING,
            RobotRecoveryState::HAS_FALLEN,
            RobotRecoveryState::IS_GETTING_UP
        };
        this->data->recoveryState = recoveryStateMap[static_cast<int>(recoveryState.state)];
        this->data->isRecoveryAvailable = static_cast<bool>(recoveryState.is_recovery_available);
        this->data->currentRobotModeIndex = static_cast<int>(recoveryState.current_planner_index);

        // cout << "recoveryState: " << static_cast<int>(recoveryState.state) << endl;
        // cout << "recovery is available: " << static_cast<int>(recoveryState.is_recovery_available) << endl;
        // cout << "current planner idx: " << static_cast<int>(recoveryState.current_planner_index) << endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}


int Brain::markCntOnFieldLine(const string markType, const FieldLine line, const double margin) {
    int cnt = 0;
    auto markings = data->getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        auto marking = markings[i];
        if (marking.label == markType) {
            Point2D point = {marking.posToField.x, marking.posToField.y};
            if (fabs(pointPerpDistToLine(point, line.posToField)) < margin) {
                cnt += 1;
            }
        }
    }
    return cnt;
}

int Brain::goalpostCntOnFieldLine(const FieldLine line, const double margin) {
    int cnt = 0;
    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++) {
        auto post = goalposts[i];
        Point2D point = {post.posToField.x, post.posToField.y};
        if (pointMinDistToLine(point, line.posToField) < margin) {
            cnt += 1;
        }
    }
    return cnt;
}

bool Brain::isBallOnFieldLine(const FieldLine line, const double margin) {
    auto ballPos = data->ball.posToField;
    Point2D point = {ballPos.x, ballPos.y};
    return fabs(pointPerpDistToLine(point, line.posToField)) < margin;
}

void Brain::identifyFieldLine(FieldLine& line) {
    auto mapLines = config->mapLines;
    FieldLine mapLine;
    double confidence;
    line.type = LineType::NA;

    double bestConfidence = 0;
    double secondBestConfidence = 0;
    int bestIndex = -1;
    for (int i = 0; i < mapLines.size(); i++) {
        mapLine = mapLines[i];
        confidence = line.dir == mapLine.dir ?
            probPartOfLine(line.posToField, mapLine.posToField)
            : 0.0;

        // Boost confidence with other features
        if (mapLine.type == LineType::GoalLine) {
            confidence += 0.3 * markCntOnFieldLine("TCross", line, 0.2);
            confidence += 0.5 * goalpostCntOnFieldLine(line, 0.2);
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "CORNER_KICK")
            ) confidence += 0.3; // 瑙掔悆鏃? 鐞冨湪搴曠嚎涓?
        }
        if (mapLine.type == LineType::MiddleLine) {
            confidence += 0.3 * markCntOnFieldLine("XCross", line, 0.2);
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "GOAL_KICK")
            ) confidence += 0.3; // 闂ㄧ悆鏃? 鐞冨湪涓嚎涓?
        }
        if (mapLine.type == LineType::TouchLine) {
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "GOAL_KICK" || data->realGameSubState == "CORNER_KICK" || data->realGameSubState == "THROW_IN")
            ) confidence += 0.3; // 鍙戣鐞? 闂ㄧ悆鍜岃竟绾跨悆鏃? 鐞冨湪杈圭嚎涓?
        }

        // 闃叉灏?goalarealine 璇涓?goalline
        auto fd = config->fieldDimensions;
        if (
            mapLine.type == LineType::GoalLine
            && fabs(line.posToField.y0) < fd.goalAreaWidth / 2 + 0.5
            && fabs(line.posToField.y1) < fd.goalAreaWidth / 2 + 0.5
        ) confidence -= 0.3;

        // 闃叉灏?penalty area 璇涓?touchline
        if (
            mapLine.type == LineType::TouchLine
            && min(fabs(line.posToField.x0), fabs(line.posToField.x1)) > fd.length / 2.0 -  fd.penaltyAreaLength - 0.5
            && line.posToField.x0 * line.posToField.x1 > 0
        ) confidence -= 0.3;

        double length = norm(line.posToField.x0 - line.posToField.x1, line.posToField.y0 - line.posToField.y1);
        if (length < 0.5) confidence -= 0.5;
        else if (length < 1.0) confidence -= 0.1;

        if (confidence > bestConfidence) {
            secondBestConfidence = bestConfidence;
            bestConfidence = confidence;
            bestIndex = i;
        }
    }

    if (bestConfidence - secondBestConfidence < 0.5) bestConfidence -= 0.5;



    if (bestIndex >= 0 && bestIndex < mapLines.size()) {
        line.type = mapLines[bestIndex].type;
        line.half = mapLines[bestIndex].half;
        line.side = mapLines[bestIndex].side;
        line.confidence = bestConfidence;
        return;
    }

    // else
    line.type = LineType::NA;
    line.half = LineHalf::NA;
    line.side = LineSide::NA;
    line.confidence = 0.0;
    return;
}

void Brain::identifyMarking(GameObject& marking) {
    double minDist = 100;
    double secMinDist = 100;
    int mmIndex = -1;
    for (int i = 0; i < config->mapMarkings.size(); i++) {
       auto mm = config->mapMarkings[i];

       if (mm.type != marking.label) continue;

       double dist = norm(marking.posToField.x - mm.x, marking.posToField.y - mm.y);

       if (dist < minDist) {
           secMinDist = minDist;
           minDist = dist;
           mmIndex = i;
       } else if (dist < secMinDist) {
           secMinDist = dist;
       }
    }

    auto fd = config->fieldDimensions;
    if (
        mmIndex >=0 && mmIndex < config->mapMarkings.size()
        && minDist < 1.5 * 14 / fd.length // 1.0 for adultsize
        && secMinDist - minDist > 1.5 * 14 / fd.length // 2.0 for adultsize
        // && marking.confidence > 70
    ) {
        marking.id = mmIndex;
        marking.name = config->mapMarkings[mmIndex].name;
        marking.idConfidence = 1.0;
    } else {
        marking.id = -1;
        marking.name = "NA";
        marking.idConfidence = 0.0;
    }
}


void Brain::identifyGoalpost(GameObject& goalpost) {
    string side = "NA";
    string half = "NA";
    if (goalpost.posToField.x > 0) half = "O";
    else half = "S";

    if (goalpost.posToField.y > 0) side = "L";
    else side = "R";

    goalpost.id = 0;
    goalpost.name = half + side;
    goalpost.idConfidence = 1.0;
    // TODO 鍙傝€?markings, 鍋氭洿涓虹簿缁嗙殑 goalpostid
}

vector<FieldLine> Brain::processFieldLines(vector<FieldLine>& fieldLines) {
    vector<FieldLine> original = fieldLines;
    vector<FieldLine> res;


    int sizeBefore = original.size();
    // merge lines that are actually the same line
    for (int i = 0; i < original.size(); i++) {
        for (int j = i + 1; j < original.size(); j++) {
            auto line1 = original[i].posToField;
            auto line2 = original[j].posToField;
            if (isSameLine(line1, line2, 0.1, 1.0)) {
                FieldLine mergedLine;
                mergedLine.posToField = mergeLines(line1, line2);
                mergedLine.posToRobot = mergeLines(original[i].posToRobot, original[j].posToRobot);
                mergedLine.posOnCam = mergeLines(original[i].posOnCam, original[j].posOnCam);
                mergedLine.timePoint = original[i].timePoint;

                // replace first line in original with merged line and remove second line
                original[i] = mergedLine;
                original.erase(original.begin() + j);
                j--;
            }
        }
    }
    int sizeAfter = original.size();
    // prtWarn(format("Merged %d lines into %d lines", sizeBefore, sizeAfter));
    // return original;
    // filter out lines that are too short and infer direction while ditch lines whose dir cannot be inferred
    double valve = 0.2;
    for (int i = 0; i < original.size(); i++) {
        auto line = original[i];
        auto lineDir = atan2(line.posToField.y1 - line.posToField.y0, line.posToField.x1 - line.posToField.x0);

        if (fabs(toPInPI(lineDir - M_PI)) < 0.1 || fabs(lineDir) < 0.1) line.dir = LineDir::Vertical;
        else if (fabs(toPInPI(lineDir - M_PI/2)) < 0.1 || fabs(toPInPI(lineDir + M_PI/2)) < 0.1) line.dir = LineDir::Horizontal;
        else continue;

        // if line is direction can be verified, check if it is long enough
        if (lineLength(line.posToField) > valve) {
            res.push_back(line);
        }
    }

    // identify each line
    for (int i = 0; i < res.size(); i++) {
        identifyFieldLine(res[i]);
    }
    return res;
}


vector<GameObject> Brain::getGameObjects(const vision_interface::msg::Detections &detections)
{
    vector<GameObject> res;

    // auto timestamp = detections.header.stamp;
    // rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);
    auto timePoint = timePointFromHeader(detections.header);

    for (int i = 0; i < detections.detected_objects.size(); i++)
    {
        auto obj = detections.detected_objects[i];
        GameObject gObj;

        gObj.timePoint = timePoint;
        gObj.label = obj.label;
        gObj.color = obj.color;

        if (obj.target_uv.size() == 2)
        { // 鍦伴潰鏍囧織鐐圭殑绮剧‘鍍忕礌浣嶇疆淇℃伅
            gObj.precisePixelPoint.x = static_cast<double>(obj.target_uv[0]);
            gObj.precisePixelPoint.y = static_cast<double>(obj.target_uv[1]);
        }

        gObj.boundingBox.xmax = obj.xmax;
        gObj.boundingBox.xmin = obj.xmin;
        gObj.boundingBox.ymax = obj.ymax;
        gObj.boundingBox.ymin = obj.ymin;
        gObj.confidence = obj.confidence;
        gObj.positionConfidence = obj.position_confidence;

        // 娣卞害浼樺厛
        // if (obj.position.size() > 0 && !(obj.position[0] == 0 && obj.position[1] == 0))
        // { // 娣卞害娴嬭窛鎴愬姛锛?浠ユ繁搴︽祴璺濅负鍑?
        //     gObj.posToRobot.x = obj.position[0];
        //     gObj.posToRobot.y = obj.position[1];
        // }
        // else
        // { // 娣卞害娴嬭窛澶辫触锛屼互鎶曞奖璺濈涓哄噯
        //     gObj.posToRobot.x = obj.position_projection[0];
        //     gObj.posToRobot.y = obj.position_projection[1];
        // } // 娉ㄦ剰锛寊 鍊兼病鏈夌敤鍒?

        // 涓嶇敤娣卞害娴嬭窛, 鐩存帴鐢ㄦ姇褰辫窛绂?
        gObj.posToRobot.x = obj.position_projection[0];
        gObj.posToRobot.y = obj.position_projection[1];

        // 璁＄畻瑙掑害
        gObj.range = norm(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = atan2(config->robotHeight, gObj.range); // 娉ㄦ剰杩欐槸涓€涓繎浼煎€?

        // 璁＄畻瀵硅薄鍦?field 鍧愭爣绯讳腑鐨勪綅缃?
        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z // 娉ㄦ剰, z 娌℃湁鍦ㄥ叾瀹冨湴鏂逛娇鐢? 杩欓噷浠呬负鍙傛暟鍗犱綅浣跨敤
        );

        res.push_back(gObj);
    }

    return res;
}

void Brain::detectProcessBalls(const vector<GameObject> &ballObjs)
{
    static rclcpp::Time lastSeenRealBallTime;
    double bestConfidence = 0;
    int indexRealBall = -1;  // 璁や负鍝竴涓悆鏄湡鐨? -1 琛ㄧず娌℃湁妫€娴嬪埌鐞?

    // 鎵惧嚭鏈€鍙兘鐨勭湡鐞?
    for (int i = 0; i < ballObjs.size(); i++)
    {
        auto ballObj = ballObjs[i];
        auto oldBall = data->ball;

        // 闃叉鎶婂ぉ涓婄殑鐏瘑鍒负鐞?
        if (ballObj.posToRobot.x < -0.5 || ballObj.posToRobot.x > 15.0)
            continue;

        // 鎺掗櫎鍦ㄥ満澶栧お杩滅殑鐞? 娉ㄦ剰杩欎釜鍔熻兘浼氬奖鍝?cahse 绛夊姛鑳? 鍏堜笉閲囩敤.
        // if (isBallOut(2.0, 2.0))
        //     continue;

        // 濡傛灉妫€娴嬪嚭鐨勭悆涓庝笂娆＄湅鍒扮殑鐞? 璺濈鍜屾椂闂撮兘寰堣繎, 鍒欏鍏?confidence 杩涜閫傚綋鍔犲垎
        // double c = ballObj.confidence;
        // double oldC = oldBall.confidence;
        // double msecs = msecsSince(oldBall.timePoint);
        // double dist = norm(ballObj.posToField.x - oldBall.posToField.x, ballObj.posToField.y - oldBall.posToField.y);
        // ballObj.confidence += 0.5 * max(oldC, 0.0) * max(1 - msecs/1000, 0.0) * max(1 - dist / 2, 0.0);
        // ballObj.confidence = min(100.0, ballObj.confidence);
        // log->setTimeNow();
        // log->log("/debug/oldc_newc", rerun::TextLog(format(
        //     "oldc: %.2f  newc: %.2f",
        //     oldC,
        //     ballObj.confidence
        // )));

        // 鍒ゆ柇: 濡傛灉缃俊搴﹀お浣? 鍒欒涓烘槸璇
        if (ballObj.confidence < config->ballConfidenceThreshold)
            continue;

        // TODO 鍔犲叆鏇村鎺掗櫎鍙傛暟, 渚嬪鍦ㄨ韩浣撲笂, 鏄庢樉鍦ㄧ悆鍦哄, 浣嶇疆绐佺劧澶у箙搴﹀彉鍖栫瓑
        // 琚伄鎸＄殑鏉′欢瑕佸姞鍏? 濡傛灉绐佺劧娑堝け, 娌℃湁閬尅鐨勮瘽, 鍒欏彧鐩镐俊涓€灏忎細鍎? 濡傛灉鏈夐伄鎸? 鍙互鐩镐俊姣旇緝闀跨殑鏃堕棿.

        // 鎵惧嚭鍓╀笅鐨勭悆涓? 缃俊搴︽渶楂樼殑
        if (ballObj.confidence > bestConfidence)
        {
            bestConfidence = ballObj.confidence;
            indexRealBall = i;
        }
    }

    auto now = this->get_clock()->now();

    if (indexRealBall >= 0)
    { // 妫€娴嬪埌鐞冧簡
        data->ballDetected = true;

        data->ball = ballObjs[indexRealBall];
        data->ball.confidence = bestConfidence;
        data->ballEffectiveConfidence = bestConfidence;
        if (ballPredictor) {
            ballPredictor->add(
                data->ball.timePoint,
                data->ball.posToField.x,
                data->ball.posToField.y,
                max(0.1, data->ball.range)
            );
        }

        tree->setEntry<bool>("ball_location_known", true);
        updateBallOut();

        lastSeenRealBallTime = now;
        data->lose_ball = false;
    }
    else
    { // 娌℃湁妫€娴嬪埌鐞?
        log->setTimeNow();
        // log->log("image/detection_boxes_realball", rerun::Clear::FLAT);
        data->ballDetected = false;
        data->ball.boundingBox.xmin = 0;
        data->ball.boundingBox.xmax = 0;
        data->ball.boundingBox.ymin = 0;
        data->ball.boundingBox.ymax = 0;

        if (lastSeenRealBallTime.seconds() > 0.0)
        {
            double msecs = (now - lastSeenRealBallTime).nanoseconds() / 1e6;
            data->lose_ball = (msecs > 2000.0);
        }
        else
        {
            data->lose_ball = false;
        }

        // data->ball.confidence = 0; // DO NOT set confidence to 0, confidence decay depends on this.
    }

    // 璁＄畻鏈哄櫒浜哄埌鐞冪殑鍚戦噺, 鍦?field 鍧愭爣绯讳腑鐨勬柟鍚?
    data->robotBallAngleToField = atan2(data->ball.posToField.y - data->robotPoseToField.y, data->ball.posToField.x - data->robotPoseToField.x);
}

void Brain::detectProcessMarkings(const vector<GameObject> &markingObjs)
{
    // // for testing 娴嬭窛绋冲畾鎬?---------
    // for (int i = 0; i < markingObjs.size(); i++) {
    //    auto m = markingObjs[i];
    //    if (m.label != "PenaltyPoint" || m.posToField.x < 0.0) continue;

    //    double range = norm(m.posToField.x - data->robotPoseToField.x, m.posToField.y - data->robotPoseToField.y);

    //    log->setTimeNow();
    //    log->log("debug/penalty_point/range", rerun::Scalar(range));
    //    log->log("debug/penalty_point/x", rerun::Scalar(m.posToField.x));
    //    log->log("debug/penalty_point/y", rerun::Scalar(m.posToField.y));
    // }
    // // end testing

    const double confidenceValve = 50; // confidence 浣庝簬杩欎釜闃堝€? 鎺掗櫎
    vector<GameObject> markings = {};
    for (int i = 0; i < markingObjs.size(); i++)
    {
        auto marking = markingObjs[i];

        // 鍒ゆ柇: 濡傛灉缃俊搴﹀お浣? 鍒欒涓烘槸璇
        if (marking.confidence < confidenceValve)
            continue;

        // 鎺掗櫎澶╃殑涓婅璇嗗埆鏍囪
        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 15.0)
            continue;

        // 濡傛灉閫氳繃浜嗛噸閲嶈€冮獙, 鍒欒鍏?brain
        identifyMarking(marking);
        markings.push_back(marking);
    }
    data->setMarkings(markings);

    // log identified markings
    log->setTimeNow();
    vector<rerun::LineStrip2D> circles = {};
    vector<string> labels = {};

    for (int i = 0; i < markings.size(); i++) {
        auto m = markings[i];
        if (markings[i].id >= 0) {
            circles.push_back(log->circle(m.posToField.x, -m.posToField.y, 0.3));
            labels.push_back(format("%s c=%.2f", m.name.c_str(), m.idConfidence));
        }
    }

    log->log("field/identified_markings",
        rerun::LineStrips2D(rerun::Collection<rerun::components::LineStrip2D>(circles))
       .with_radii(0.01)
       .with_labels(labels)
       .with_colors(0xFFFFFFFF));
}

void Brain::detectProcessGoalposts(const vector<GameObject> &goalpostObjs)
{
    const double confidenceValve = 50; // confidence 浣庝簬杩欎釜闃堝€? 鎺掗櫎
    vector<GameObject> goalposts = {};

    for (int i = 0; i < goalpostObjs.size(); i++) {
        auto goalpost = goalpostObjs[i];

        // 鍒ゆ柇: 濡傛灉缃俊搴﹀お浣? 鍒欒涓烘槸璇
        if (goalpost.confidence < confidenceValve)
            continue;

        identifyGoalpost(goalpost);
        goalposts.push_back(goalpost);
    }
    data->setGoalposts(goalposts);

    // log identified goalposts
    log->setTimeNow();
    vector<rerun::LineStrip2D> circles = {};
    vector<string> labels = {};

    for (int i = 0; i < goalposts.size(); i++) {
        auto p = goalposts[i];
        if (goalposts[i].id >= 0) {
            circles.push_back(log->circle(p.posToField.x, -p.posToField.y, 0.3));
            labels.push_back(format("%s c=%.2f", p.name.c_str(), p.idConfidence));
        }
    }

    // log->log("field/identified_goalposts",
    //     rerun::LineStrips2D(rerun::Collection<rerun::components::LineStrip2D>(circles))
    //     .with_radii(0.01)
    //     .with_labels(labels)
    //     .with_colors(0xFFFFFFFF));
}


void Brain::detectProcessRobots(const vector<GameObject> &robotObjs) {
    // auto robots = data->getRobots();

    // for (int i = 0; i < robotObjs.size(); i++) {
    //     auto rbt = robotObjs[i];
    //     if (rbt.confidence < 50) continue;

    //     // find nearest robot in memory
    //     double minDist = 1e6;
    //     int minIndex = -1;
    //     for (int j = 0; j < robots.size(); j++) {
    //         auto rm = robots[j];
    //         double dist = norm(rm.posToField.x - rbt.posToField.x, rm.posToField.y - rbt.posToField.y);
    //         if (dist < minDist) {
    //             minDist = dist;
    //             minIndex = j;
    //         }
    //     }
    //     // prtDebug(format("minDist = %.2f", minDist));
    //     if (minDist < 0.5) { // 璁や负鏄悓涓€涓満鍣ㄤ汉
    //         robots[minIndex] = rbt;
    //     } else { // 璁や负鏄笉鍚岀殑鏈哄櫒浜?
    //         robots.push_back(rbt);
    //     }
    // }
    // // 娉ㄦ剰杩欓噷涓嶆竻鐞嗗凡缁忕湅涓嶈鐨勬満鍣ㄤ汉, 鑰屾槸鍦?updateMemory 涓繘琛屽鐞?

    vector<GameObject> robots = {};
    for (int i = 0; i < robotObjs.size(); i++) {
        auto rbt = robotObjs[i];
        if (rbt.confidence < 50) continue;

        // else
        robots.push_back(rbt);
    }

    data->setRobots(robots);
}


void Brain::detectProcessVisionBox(const vision_interface::msg::Detections &msg) {
    // auto detection_time_stamp = msg.header.stamp;
    // rclcpp::Time timePoint(detection_time_stamp.sec, detection_time_stamp.nanosec);
    auto timePoint = timePointFromHeader(msg.header);

    // 澶勭悊骞惰褰曡閲庝俊鎭?
    VisionBox vbox;
    vbox.timePoint = timePoint;
    for (int i = 0; i < msg.corner_pos.size(); i++) vbox.posToRobot.push_back(msg.corner_pos[i]);

    // 澶勭悊宸︿笂涓庡彸涓婁袱鐐?x 灏忎簬 0 , 瀹為檯涓烘棤闄愯繙鐨勫満鏅?
    const double VISION_LIMIT = 20.0;
    vector<vector<double>> v = {};
    for (int i = 0; i < 4; i++) {
        int start = i; int end = (i + 1) % 4;
        v.push_back({vbox.posToRobot[end * 2] - vbox.posToRobot[start * 2], vbox.posToRobot[end * 2 + 1] - vbox.posToRobot[start * 2 + 1]});
        v.push_back({-vbox.posToRobot[end * 2] + vbox.posToRobot[start * 2], -vbox.posToRobot[end * 2 + 1] + vbox.posToRobot[start * 2 + 1]});
    }

    for (int i = 0; i < 2; i++) {
        double ox = vbox.posToRobot[2* i]; double oy = vbox.posToRobot[2 * i + 1];
        if (
            (i == 0 && crossProduct(v[5], v[6]) < 0)
            || (i == 1 && crossProduct(v[3], v[4]) < 0)
        ){
            vbox.posToRobot[2 * i] = -ox / fabs(ox) * VISION_LIMIT;
            vbox.posToRobot[2 * i + 1] = -oy / fabs(oy) * VISION_LIMIT;
        }
    }

    // 杞崲鍒?field 鍧愭爣绯讳腑
    for (int i = 0; i < 5; i++) {
        double xr, yr, xf, yf, __;
        xr = vbox.posToRobot[2 * i];
        yr = vbox.posToRobot[2 * i + 1];
        transCoord(
            xr, yr, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            xf, yf, __
        );
        vbox.posToField.push_back(xf);
        vbox.posToField.push_back(yf);
    }

    // 涓€娆℃€у皢缁撴灉璧嬪€煎埌 data 涓?
    data->visionBox = vbox;
}

void Brain::logVisionBox(const rclcpp::Time &timePoint) {
    if (data->visionBox.posToField.size() >= 10) {
        auto v = data->visionBox.posToField;

        rerun::LineStrip2D logLines({{v[0], -v[1]}, {v[2], -v[3]}, {v[4], -v[5]}, {v[6], -v[7]}, {v[0], -v[1]}});
        log->setTimeSeconds(timePoint.seconds());
        log->log(
            "field/visionBox",
            rerun::LineStrips2D(logLines)
            .with_colors(0x0000FFCC)
            .with_radii(0.01)
        );
    }
}

void Brain::logDetection(const vector<GameObject> &gameObjects, bool logBoundingBox) {
    if (gameObjects.size() == 0) {
        if (logBoundingBox) log->log("image/detection_boxes", rerun::Clear::FLAT);
        log->log("field/detection_points", rerun::Clear::FLAT);
        // log->log("robotframe/detection_points", rerun::Clear::FLAT);
        return;
    }

    // else
    rclcpp::Time timePoint = gameObjects[0].timePoint;
    log->setTimeSeconds(timePoint.seconds());

    map<std::string, rerun::Color> detectColorMap = {
        {"LCross", rerun::Color(0xFFFF00FF)},
        {"TCross", rerun::Color(0x00FF00FF)},
        {"XCross", rerun::Color(0x0000FFFF)},
        {"Person", rerun::Color(0xFF00FFFF)},
        {"Goalpost", rerun::Color(0x00FFFFFF)},
        {"Opponent", rerun::Color(0xFF0000FF)},
        {"PenaltyPoint", rerun::Color(0xFF9900FF)},
    };

    // for logging boundingBoxes
    vector<rerun::Vec2D> mins;
    vector<rerun::Vec2D> sizes;
    vector<rerun::Text> labels;
    vector<rerun::Color> colors;

    // for logging marker points in robot frame
    vector<rerun::Vec2D> points;
    vector<rerun::Vec2D> points_r; // robot frame
    vector<double> radiis;

    auto projectionModeToString = [](int mode) -> const char * {
        switch (mode) {
            case 1:
                return "refined_plane";
            case 2:
                return "hold_last_valid";
            case 3:
                return "fallback_z0";
            default:
                return "unknown";
        }
    };

    for (int i = 0; i < gameObjects.size(); i++)
    {
        auto obj = gameObjects[i];
        auto label = obj.label;
        string displayLabel = label;
        if (label == "Ball") {
            displayLabel += "[" + string(projectionModeToString(obj.positionConfidence)) + "]";
        } else if (label == "Opponent" || label == "Person") {
            displayLabel += "[" + obj.color + "]";
        }
        labels.push_back(rerun::Text(
            format("%s x:%.2f y:%.2f c:%.1f",
                displayLabel.c_str(),
                obj.posToRobot.x,
                obj.posToRobot.y,
                obj.confidence)
            )
        );
        points.push_back(rerun::Vec2D{obj.posToField.x, -obj.posToField.y}); // y 鍙栧弽鏄洜涓?rerun Viewer 鐨勫潗鏍囩郴鏄乏鎵嬬郴銆傝浆涓€涓嬬湅璧锋潵鏇存柟渚裤€?
        points_r.push_back(rerun::Vec2D{obj.posToRobot.x, -obj.posToRobot.y});
        mins.push_back(rerun::Vec2D{obj.boundingBox.xmin, obj.boundingBox.ymin});
        sizes.push_back(rerun::Vec2D{obj.boundingBox.xmax - obj.boundingBox.xmin, obj.boundingBox.ymax - obj.boundingBox.ymin});

        // if (obj.label == "Opponent") radiis.push_back(0.5);
        radiis.push_back(0.1);

        auto color = rerun::Color(0xFFFFFFFF);

        auto it = detectColorMap.find(label);
        if (it != detectColorMap.end())
        {
            color = detectColorMap[label];
        }
        else
        {
            // do nothing, use default
            // colors.push_back(rerun::Color(0xFFFFFFFF));
        }
        if (label == "Ball" && isBallOut(0.2, 10.0))
            color = rerun::Color(0x000000FF);
        if (label == "Ball" && obj.confidence < config->ballConfidenceThreshold)
            color = rerun::Color(0xAAAAAAFF);
        colors.push_back(color);
    }


    if (logBoundingBox) log->log("image/detection_boxes",
             rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                 .with_labels(labels)
                 .with_colors(colors));

    log->log("field/detection_points",
             rerun::Points2D(points)
                 .with_colors(colors)
                 .with_radii(radiis)
             // .with_labels(labels)
    );
    // log->log("robotframe/detection_points",
    //          rerun::Points2D(points_r)
    //              .with_colors(colors)
    //              .with_radii(radiis)
    //          // .with_labels(labels)
    // );
}


void Brain::logMemRobots() {
    auto rbts = data->getRobots();
    // prtDebug(format("logMemRobots called, robotsize = %d", rbts.size()), RED_CODE);

    if (rbts.size() == 0) {
        log->log("field/mem_robots", rerun::Clear::FLAT);
        // log->log("robotframe/mem_robots", rerun::Clear::FLAT);
        return;
    }

    // else
    log->setTimeNow();
    // vector<rerun::Vec2D> points;
    vector<rerun::LineStrip2D> circles;
    vector<rerun::Vec2D> points_r; // robot frame
    for (int i = 0; i < rbts.size(); i++)
    {
        auto rbt = rbts[i];
        log->logRobot("field/robots", Pose2D({rbt.posToField.x, rbt.posToField.y, -M_PI}), 0xFF0000FF);
        // circles.push_back(log->circle(rbt.posToField.x, -rbt.posToField.y, 0.5)); // y 鍙栧弽鏄洜涓?rerun Viewer 鐨勫潗鏍囩郴鏄乏鎵嬬郴銆傝浆涓€涓嬬湅璧锋潵鏇存柟渚裤€?
        // points_r.push_back(rerun::Vec2D{rbt.posToRobot.x, -rbt.posToRobot.y});
    }

    // log->log("field/mem_robots",
    //          rerun::LineStrips2D(circles)
    //              .with_colors(0xFF0000AA)
    //              .with_radii(0.01)
    //          // .with_labels(labels)
    // );
    // log->log("robotframe/mem_robots",
    //          rerun::Points2D(points_r)
    //              .with_colors(0xFF0000AA)
    //              .with_radii(0.5)
    //          // .with_labels(labels)
    // );
}

void Brain::logObstacles() {
    // log->setTimeNow();
    // time is set on the outside

    // 璁板綍闅滅鐗?鍗虫湁琚崰鐢ㄧ殑缃戞牸)
    auto obs = data->getObstacles();
    vector<rerun::Vec2D> points;
    vector<rerun::Color> colors;
    vector<rerun::Text> labels;
    const int occThreshold = get_parameter("obstacle_avoidance.occupancy_threshold").as_int();
    for (int i = 0; i < obs.size(); i++) {
        auto o = obs[i];

        if (o.confidence < occThreshold) continue; // 杩欎釜閫昏緫瑕嗙洊浜嗗悗闈㈢殑閫昏緫, 娉ㄦ帀鍙互浠ヤ笉鍚岄鑹?log 涓嶅悓鐨勭疆淇″害.

        points.push_back(rerun::Vec2D{o.posToField.x, -o.posToField.y});
        double mem_msecs = get_parameter("obstacle_avoidance.obstacle_memory_msecs").as_double();
        auto age = msecsSince(o.timePoint);
        uint8_t alpha = static_cast<uint8_t>(0xFF - 0xFF * age / mem_msecs);
        uint32_t color = (o.confidence > occThreshold) ? (0xFF000000 | alpha) : (0xFFFF0000 | alpha);
        colors.push_back(rerun::Color(color));

        labels.push_back(rerun::Text(format("count: %.0f age: %.0fms", o.confidence, age)));
    }
    log->log(
        "field/obstacles",
        rerun::Points2D(points)
        .with_colors(colors)
        .with_labels(labels)
        .with_radii(0.1)
    );
}

void Brain::logDepth(int grid_x_count, int grid_y_count, vector<vector<int>> &grid_occupied, vector<rerun::Vec3D> &points_robot) {
    // time is set on the outside
    const double grid_size = get_parameter("obstacle_avoidance.grid_size").as_double();  // 缃戞牸澶у皬
    const double x_min = 0.0, x_max = get_parameter("obstacle_avoidance.max_x").as_double();
    const double y_min = -get_parameter("obstacle_avoidance.max_y").as_double();
    const double y_max = -y_min;

    // 璁板綍鍘熷鐐逛簯鍜岀綉鏍?
    vector<rerun::Position3D> vertices;
    vector<rerun::Color> vertex_colors;
    vector<array<uint32_t, 3>> triangle_indices;
    const int OCCUPANCY_THRESHOLD = get_parameter("obstacle_avoidance.occupancy_threshold").as_int(); // 璁剧疆涓€涓樉绀虹敤鐨勯槇鍊?

    for (int i = 0; i < grid_x_count; i++) {
        for (int j = 0; j < grid_y_count; j++) {
            if (grid_occupied[i][j] > 0) {
                // 璁＄畻鏈夐殰纰嶇綉鏍肩殑鍥涗釜椤剁偣鍧愭爣
                double x0 = x_min + i * grid_size;
                double y0 = y_min + j * grid_size;
                double x1 = x0 + grid_size;
                double y1 = y0 + grid_size;

                // 娣诲姞鍥涗釜椤剁偣
                uint32_t base_index = vertices.size();
                vertices.push_back({x0, y0, 0.0});
                vertices.push_back({x1, y0, 0.0});
                vertices.push_back({x1, y1, 0.0});
                vertices.push_back({x0, y1, 0.0});

                // 璁剧疆棰滆壊锛氭牴鎹崰鐢ㄦ儏鍐佃缃笉鍚岀殑绾㈣壊
                rerun::Color color;
                if (grid_occupied[i][j] > OCCUPANCY_THRESHOLD) {
                    color = rerun::Color(255, 0, 0, 255);  // RGBA, 绾㈣壊
                } else {
                    color = rerun::Color(255, 255, 0, 255);  // RGBA, 榛勮壊
                }
                vertex_colors.push_back(color);
                vertex_colors.push_back(color);
                vertex_colors.push_back(color);
                vertex_colors.push_back(color);

                // 娣诲姞涓や釜涓夎褰㈤潰
                triangle_indices.push_back({base_index, base_index + 1, base_index + 2});
                triangle_indices.push_back({base_index, base_index + 2, base_index + 3});
            }
        }
    }

    vector<uint32_t> point_colors;
    for (auto &point : points_robot) {
        float z_val = std::clamp(point.z(), 0.0f, 1.0f);
        const double obstacleMinHeight = get_parameter("obstacle_avoidance.obstacle_min_height").as_double();
        if (z_val < obstacleMinHeight) {
            point_colors.push_back(0x0000FFFF);
        } else {
            uint8_t r = static_cast<uint8_t>(z_val * 255);
            uint8_t g = static_cast<uint8_t>((1 - z_val) * 255);
            point_colors.push_back((r << 24) | (g << 16) | 0xFF);
        }
    }

    log->log("depth/depth_points",
            rerun::Points3D(points_robot)
                .with_radii(0.01)
                .with_colors(point_colors)
    );

    log->log("depth/grid_mesh",
            rerun::Mesh3D(vertices)
                .with_vertex_colors(vertex_colors)
                .with_triangle_indices(triangle_indices)
    );

    // 璁板綍 ball exclusion box
    double r = get_parameter("obstacle_avoidance.ball_exclusion_radius").as_double();
    double h = get_parameter("obstacle_avoidance.ball_exclusion_height").as_double();
    log->log(
        "depth/ball_exclusion_box",
        rerun::Boxes3D::from_centers_and_half_sizes(
            {{ data->ball.posToRobot.x, data->ball.posToRobot.y, h/2}},
            {{ r, r, h/2}})
        .with_colors(0x00FF0044)     // 鍗婇€忔槑缁胯壊
    );
}

void Brain::logDebugInfo() {
    auto log_ = [=](string msg) {
        log->setTimeNow();
        log->log("debug/brain_tick", rerun::TextLog(msg));
    };
    string gameState = tree->getEntry<string>("gc_game_state");
    string gameSubState = tree->getEntry<string>("gc_game_sub_state");
    string gameSubStateType = tree->getEntry<string>("gc_game_sub_state_type");
    string isLead = data->tmImLead ? "ON" : "OFF";
    string ballOut = tree->getEntry<bool>("ball_out") ? "YES" : "NO";
    string ballDetected = data->ballDetected ? "YES" : "NO";
    string decision = tree->getEntry<string>("decision");
    string freeKickKickingOff = data->isFreekickKickingOff ? "YES" : "NO";
    string directShoot = data->isDirectShoot ? "YES" : "NO";
    string primaryStriker = isPrimaryStriker() ? "YES" : "NO";
    log_(format("Game State: %s, SubState: %s, SubStateType: %s, Lead: %s, Decision: %s, FreeKickKickingOff: %s, DirectShoot: %s, PrimaryStriker: %s",
        gameState.c_str(), gameSubState.c_str(), gameSubStateType.c_str(), isLead.c_str(), decision.c_str(), freeKickKickingOff.c_str(), directShoot.c_str(), primaryStriker.c_str()));

    log->setTimeNow();
    log->log("debug/my_cost_scalar", rerun::Scalar(data->tmMyCost));
    log->log("debug/my_lead_scalar", rerun::Scalar(data->tmImLead));
}

void Brain::updateRelativePos(GameObject &obj) {
    Pose2D pf;
    pf.x = obj.posToField.x;
    pf.y = obj.posToField.y;
    pf.theta = 0;
    Pose2D pr = data->field2robot(pf);
    obj.posToRobot.x = pr.x;
    obj.posToRobot.y = pr.y;
    obj.range = norm(obj.posToRobot.x, obj.posToRobot.y);
    obj.yawToRobot = atan2(obj.posToRobot.y, obj.posToRobot.x);
    obj.pitchToRobot = asin(config->robotHeight / obj.range);
}

void Brain::updateFieldPos(GameObject &obj) {
    Pose2D pr;
    pr.x = obj.posToRobot.x;
    pr.y = obj.posToRobot.y;
    pr.theta = 0;
    Pose2D pf = data->robot2field(pr);
    obj.posToField.x = pf.x;
    obj.posToField.y = pf.y;
    obj.range = norm(obj.posToRobot.x, obj.posToRobot.y);
    obj.yawToRobot = atan2(obj.posToRobot.y, obj.posToRobot.x);
    obj.pitchToRobot = asin(config->robotHeight / obj.range);
}

void Brain::depthImageCallback(const sensor_msgs::msg::Image &msg)
{
    try {
        // 妫€鏌ュ浘鍍忔暟鎹槸鍚︽湁鏁?
        if (msg.data.empty() || msg.height == 0 || msg.width == 0) {
            RCLCPP_WARN(get_logger(), "Received empty depth image");
            return;
        }

        // 鍒涘缓娣卞害鍥惧儚鍜岃浆鎹?
        cv::Mat depthFloat;
        // 鏍规嵁鍥惧儚缂栫爜鏍煎紡杩涜澶勭悊
        if (msg.encoding == "16UC1" || msg.encoding == "mono16") {
            size_t expected = (size_t)msg.width * msg.height * sizeof(uint16_t);
            if (msg.data.size() < expected) {
                RCLCPP_ERROR(get_logger(), "Depth mono16 size mismatch");
                return;
            }
            cv::Mat depthRaw(msg.height, msg.width, CV_16UC1, const_cast<uint8_t*>(msg.data.data()));
            depthRaw.convertTo(depthFloat, CV_32FC1, 1.0 / 1000.0); // 鑻ユ槸瀹為檯娣卞害鍗曚綅 mm
        } else if (msg.encoding == "32FC1") {
            // 妫€鏌ユ暟鎹ぇ灏忔槸鍚︽纭?
            size_t expected_size = msg.height * msg.width * sizeof(float);
            if (msg.data.size() != expected_size) {
                RCLCPP_ERROR(get_logger(), "Depth image size mismatch: expected %zu, got %zu",
                    expected_size, msg.data.size());
                return;
            }

            // 鐩存帴鍒涘缓 32 浣嶆诞鐐规暟鏍煎紡鐨勬繁搴﹀浘鍍?
            depthFloat = cv::Mat(msg.height, msg.width, CV_32FC1,
                const_cast<float*>(reinterpret_cast<const float*>(msg.data.data()))).clone();

        } else {
            RCLCPP_ERROR(get_logger(), "Unsupported depth image encoding: %s", msg.encoding.c_str());
            return;
        }

        vector<rerun::Vec3D> points_robot;  // for log

        const double fx = config->camfx;
        const double fy = config->camfy;
        const double cx = config->camcx;
        const double cy = config->camcy;
        // cout << "fx = " << fx << " fy = " << fy << " cx = " << cx << " cy = " << cy << endl;

        // 瀹氫箟缃戞牸鍙傛暟
        const double grid_size = get_parameter("obstacle_avoidance.grid_size").as_double();  // 缃戞牸澶у皬
        const double x_min = 0.0, x_max = get_parameter("obstacle_avoidance.max_x").as_double();
        const double y_min = -get_parameter("obstacle_avoidance.max_y").as_double();
        const double y_max = -y_min;
        const int grid_x_count = static_cast<int>((x_max - x_min) / grid_size);
        const int grid_y_count = static_cast<int>((y_max - y_min) / grid_size);

        // 鍒涘缓缃戞牸鍗犵敤鏁扮粍
        vector<vector<int>> grid_occupied(grid_x_count, vector<int>(grid_y_count, 0));

        // 澶勭悊娣卞害鍥惧儚鐐?
        const int sampleStep = get_parameter("obstacle_avoidance.depth_sample_step").as_int();
        for (int y = 0; y < msg.height; y += sampleStep)
        {
            for (int x = 0; x < msg.width; x += sampleStep)
            {
                float depth = depthFloat.at<float>(y, x);
                if (depth > 0)
                {
                    // 杞崲鍒扮浉鏈哄潗鏍囩郴
                    double x_cam = (x - cx) * depth / fx;
                    double y_cam = (y - cy) * depth / fy;
                    double z_cam = depth;

                    // 杞崲鍒版満鍣ㄤ汉鍧愭爣绯?
                    Eigen::Vector4d point_cam(x_cam, y_cam, z_cam, 1.0);
                    Eigen::Vector4d point_robot = data->camToRobot * point_cam;

                    // 璁板綍鐐圭敤浜庡彲瑙嗗寲
                    points_robot.push_back(rerun::Vec3D{point_robot(0), point_robot(1), point_robot(2)});

                    // 鏇存柊缃戞牸鍗犵敤鎯呭喌
                    const double Z_THRESHOLD = get_parameter("obstacle_avoidance.obstacle_min_height").as_double();
                    const double EXCLUDE_MAX_X = get_parameter("obstacle_avoidance.exclusion_x").as_double(); // 鎺掗櫎鏈哄櫒浜鸿嚜宸辩殑韬綋
                    const double EXCLUDE_MIN_X = -EXCLUDE_MAX_X;
                    const double EXCLUDE_MAX_Y = get_parameter("obstacle_avoidance.exclusion_y").as_double(); // 鎺掗櫎鏈哄櫒浜鸿嚜宸辩殑韬綋
                    const double EXCLUDE_MIN_Y = -EXCLUDE_MAX_Y;

                    auto isInRange = [&]() {
                        return point_robot(0) >= x_min && point_robot(0) < x_max
                            && point_robot(1) >= y_min && point_robot(1) < y_max;
                    };
                    auto isSelfBody = [&]() {
                        return point_robot(0) >= EXCLUDE_MIN_X && point_robot(0) <= EXCLUDE_MAX_X
                            && point_robot(1) >= EXCLUDE_MIN_Y && point_robot(1) <= EXCLUDE_MAX_Y;
                    };
                    auto isBall = [&]() {
                        double r = get_parameter("obstacle_avoidance.ball_exclusion_radius").as_double();
                        double h = get_parameter("obstacle_avoidance.ball_exclusion_height").as_double();
                        return fabs(point_robot(0) - data->ball.posToRobot.x) < r
                            && fabs(point_robot(1) - data->ball.posToRobot.y) < r
                            && point_robot(2) < h;
                    };

                    if (
                        point_robot(2) > Z_THRESHOLD
                        && isInRange()
                        &&!isSelfBody()
                        &&!isBall()
                    )
                    {
                        int grid_x = static_cast<int>((point_robot(0) - x_min) / grid_size);
                        int grid_y = static_cast<int>((point_robot(1) - y_min) / grid_size);
                        grid_occupied[grid_x][grid_y] += 1;
                    }
                }
            }
        }

        auto obs_old = data->getObstacles();
        vector<GameObject> obs_new = {};

        // 鏈鐪嬪埌鐨勮鍏?obstables
        for (int i = 0; i < grid_x_count; i++) {
            for (int j = 0; j < grid_y_count; j++) {
                if (grid_occupied[i][j] > 0) {
                    GameObject obj;
                    obj.label = "Obstacle";
                    obj.timePoint = get_clock()->now();
                    obj.posToRobot.x = x_min + (i + 0.5) * grid_size;
                    obj.posToRobot.y = y_min + (j + 0.5) * grid_size;
                    obj.confidence = grid_occupied[i][j];
                    updateFieldPos(obj);
                    obs_new.push_back(obj);
                }
            }
        }

        // 娓呯悊鏃?obstacle
        for (int i = 0; i < obs_old.size(); i++) {
           // 鍏堟妸褰撳墠瑙嗛噹鑼冨洿鍐呯殑鏃?obstacle 娓呯┖, 娉ㄦ剰瑙掑害鍙槸绮楃暐璁＄畻, 骞堕€氳繃 offset 閫傚綋鎵╁ぇ浜嗕竴浜涜寖鍥?
            double visionLeft = data->headYaw + config->camAngleX / 2;
            double visionRight = data->headYaw - config->camAngleX / 2;
            auto obs = obs_old[i];
            const double offset = 0.20;
            double obsYawLeft = atan2(obs.posToRobot.y - offset, obs.posToRobot.x + offset);
            double obsYawRight = atan2(obs.posToRobot.y + offset, obs.posToRobot.x + offset);
            if (obsYawLeft < visionLeft && obsYawRight > visionRight) continue;

            // 濡傛灉鏃х殑 obs 涓庢柊鐨?obs 澶繃鎺ヨ繎, 鍒欒涓烘棫鐨?obs 宸茬粡涓嶅瓨鍦? 闃叉杈圭晫鎯呭喌涓?obs 鍫嗙Н
            bool found = false;
            for (int j = 0; j < obs_new.size(); j++) {
                auto obs_n = obs_new[j];
                double dist = norm(obs.posToRobot.x - obs_n.posToRobot.x, obs.posToRobot.y - obs_n.posToRobot.y);
                if (dist < 0.5 * grid_size) {
                    found = true;
                    break;
                }
            }
            if (found) continue;

            // else
            obs_new.push_back(obs);
        }


        data->setObstacles(obs_new); // note: 姝ゅ涓嶆竻绌鸿秴鏃剁殑鏃?obstacles, 鑰屽湪 tick 涓竻鐞?
        log->setTimeSeconds(timePointFromHeader(msg.header).seconds());
        logDepth(grid_x_count, grid_y_count, grid_occupied, points_robot);
        logObstacles();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception in depth image callback: %s", e.what());
    }
}

double Brain::distToObstacle(double angle) {
    auto obs = data->getObstacles();
    double minDist = 1e9;
    // double obstacleThreshold = static_cast<double>(config->obstacleThreshold);
    double obstacleThreshold = static_cast<double>(get_parameter("obstacle_avoidance.occupancy_threshold").as_int());

    double collisionThreshold = config->collisionThreshold;

    for (int i = 0; i < obs.size(); i++) {
        if (obs[i].confidence < obstacleThreshold) continue;

        auto o = obs[i];
        Line line = {
            0, 0,
            cos(angle) * 100, sin(angle) * 100
        };
        double perpDist = fabs(pointPerpDistToLine(Point2D{o.posToRobot.x, o.posToRobot.y}, line));
        if (perpDist < collisionThreshold) {
            double dist = innerProduct(vector<double>{o.posToRobot.x, o.posToRobot.y}, vector<double>{cos(angle), sin(angle)});
            if (dist > 0 && dist < minDist) {
                minDist = dist;
            }
        }
    }
    return minDist;
}

vector<double> Brain::findSafeDirections(double startAngle, double safeDist, double step) {
    double safeAngleLeft = startAngle;
    double safeAngleRight = startAngle;
    double leftFound = 0;
    double rightFound = 0;
    for (double angle = startAngle; angle < startAngle + M_PI; angle += step) {
        if (distToObstacle(angle) > safeDist) {
            safeAngleLeft = angle;
            leftFound = 1;
            break;
        }
    }
    for (double angle = startAngle; angle > startAngle - M_PI; angle -= step) {
        if (distToObstacle(angle) > safeDist) {
            safeAngleRight = angle;
            rightFound = 1;
            break;
        }
    }

    return vector<double>{leftFound, toPInPI(safeAngleLeft), rightFound, toPInPI(safeAngleRight)};
}

double Brain::calcAvoidDir(double startAngle, double safeDist) {
    auto res = findSafeDirections(startAngle, safeDist);
    bool leftFound = res[0] > 0.5;
    bool rightFound = res[2] > 0.5;
    double angleLeft = res[1];
    double angleRight = res[3];
    double determinedAngle = 0;
    if (leftFound && rightFound) {
        determinedAngle = fabs(angleLeft) < fabs(angleRight) ? angleLeft : angleRight;
    } else if (leftFound) {
        determinedAngle = angleLeft;
    } else if (rightFound) {
        determinedAngle = angleRight;
    } else {
        return 0;
    }
    return toPInPI(determinedAngle);
}

void Brain::updateLogFile() {
    if (config->rerunLogEnableFile && msecsSince(data->timeLastLogSave) > config->rerunLogMaxFileMins * 60000)
        log->updateLogFilePath();
}

// ------------------------------------------------------ 璋冭瘯 log 鐩稿叧 ------------------------------------------------------
void Brain::logObstacleDistance() {
    log->setTimeNow();

    // log obstacle distance for test
    vector<rerun::LineStrip2D> lines = {};
    for (int i = 0; i < 180; i++) {
        double angle = i * M_PI / 90;
        double dist = min(5.0, distToObstacle(angle));
        double angle_f = toPInPI(data->robotPoseToField.theta + angle);
        lines.push_back(
            rerun::LineStrip2D({
                {data->robotPoseToField.x, -data->robotPoseToField.y},
                {data->robotPoseToField.x + cos(-angle_f) * dist, -data->robotPoseToField.y + sin(-angle_f) * dist}
            })
        );
    }
    log->log(
        "field/obstacle_distance",
        rerun::LineStrips2D(lines)
            .with_colors(0x666666FF)
            .with_radii(0.01)
            .with_draw_order(-10)
    );
}

void Brain::logLags() {
    log->setTimeNow();
    auto color = 0x00FF00FF;

    double detLag = msecsSince(data->timeLastDet);
    if (detLag > 500) color = 0xFF0000FF;
    else if (detLag > 100) color = 0xFFFF00FF;
    else  color = 0x00FF00FF;
    double MAX_LAG_LENGTH = config->camPixX;
    log->log(
        "image/detection_lag",
        rerun::LineStrips2D(rerun::LineStrip2D({{10., -150.}, {10. + min(detLag, MAX_LAG_LENGTH), -150.}}))
            .with_colors(color)
            .with_radii(2.0)
            .with_draw_order(10)
            .with_labels({format("Det Lag %.0fms", detLag)})
    );
    log->log(
        "performance/detection_lag_timeseries",
        rerun::Scalar(detLag)
    );

    // log fieldline detection delay
    double lineLag = msecsSince(data->timeLastLineDet);
    if (lineLag > 500) color = 0xFF0000FF;
    else if (lineLag > 100) color = 0xFFFF00FF;
    else  color = 0x00FF00FF;

    log->log(
        "image/fieldline_detection_lag",
        rerun::LineStrips2D(rerun::LineStrip2D({{10., -100.}, {10. + min(lineLag, MAX_LAG_LENGTH), -100.}}))
            .with_colors(color)
            .with_radii(2.0)
            .with_draw_order(10)
            .with_labels({format("Line Det Lag %.0fms", lineLag)})
    );
    log->log(
        "performance/fieldline_detection_lag_timeseries",
        rerun::Scalar(lineLag)
    );

    // log game control delay
    double gcLag = msecsSince(data->timeLastGamecontrolMsg);
    if (gcLag > 5000) color = 0xFF0000FF;
    else if (gcLag > 1000) color = 0xFFFF00FF;
    else color = 0x00FF00FF;

    log->log(
        "image/gamecontrol_lag",
            rerun::LineStrips2D(rerun::LineStrip2D({{10., -50.}, {10. + min(gcLag, MAX_LAG_LENGTH), -50.}}))
            .with_colors(color)
            .with_radii(2.0)
            .with_draw_order(10)
            .with_labels({format("GC Lag %.0fms", gcLag)})
    );
    log->log(
        "performance/gamecontrol_lag_timeseries",
        rerun::Scalar(gcLag)
    );
}

void Brain::statusReport() {
    string decision = tree->getEntry<string>("decision");
    string playerRole = tree->getEntry<string>("player_role");
    string goalieMode = tree->getEntry<string>("goalie_mode");
    int controlState = tree->getEntry<int>("control_state");
    bool ballKnown = tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = tree->getEntry<bool>("tm_ball_pos_reliable");
    bool waitForOpponentKickoff = tree->getEntry<bool>("wait_for_opponent_kickoff");
    bool isUnderPenalty = tree->getEntry<bool>("gc_is_under_penalty");
    int robotStateCode = getRobotStateCode();
    string robotState = getRobotStateText();

    log->setTimeNow();
    log->log("debug/robot_state", rerun::TextLog(format("state=%s code=%s role=%s decision=%s control_state=%d ball_known=%d ball_detected=%d tm_ball_pos_reliable=%d wait_for_opponent_kickoff=%d goalie_mode=%s under_penalty=%d", robotState.c_str(), robotStateCodeName(robotStateCode).c_str(), playerRole.c_str(), decision.c_str(), controlState, ballKnown, data->ballDetected, tmBallPosReliable, waitForOpponentKickoff, goalieMode.c_str(), isUnderPenalty)));

    if (!config->soundEnable || config->soundPack != "espeak") return;

    string robotStateSpeech = getRobotStateSpeech(robotStateCode);
    static string lastRobotStateReport = "";
    bool robotStateSpoken = false;
    if (lastRobotStateReport != robotState) {
        if (config->soundDebugLogs) {
            log->setTimeNow();
            log->log("debug/robot_state_speech", rerun::TextLog(format("state=%s speech=%s", robotState.c_str(), robotStateSpeech.c_str())));
        }
        robotStateSpoken = speak(robotStateSpeech);
        if (robotStateSpoken) lastRobotStateReport = robotState;
    }

    static int reportInterval = 100;
    static string lastReport = "";
    string report;
    bool camOK = msecsSince(data->timeLastDet) < 1000;
    bool gcOK = msecsSince(data->timeLastGamecontrolMsg) < 1000;

    if (camOK && gcOK) {
        report = "Team" + to_string(config->teamId) + " Player " + to_string(config->playerId) + " " + tree->getEntry<string>("player_role") + " " + " OK";
    } else {
        report = "";
        if (!camOK) report += "camera lost";
        if (!gcOK) report += "gamecontrol lost";
    }
    if (!robotStateSpoken && lastReport != report && speak(report)) lastReport = report;
}

void Brain::logStatusToConsole() {
    static int cnt = 0;
    const int LOG_INTERVAL = 30;
    cnt++;
    if (cnt % LOG_INTERVAL == 0) {
        string msg = "";
        string gameState = tree->getEntry<string>("gc_game_state");
        gameState = gameState == "" ? "-----" : gameState;
        string gameSubType = tree->getEntry<string>("gc_game_sub_state_type");
        gameSubType = gameSubType == "" ? "-----" : gameSubType;
        string gameSubState = tree->getEntry<string>("gc_game_sub_state");
        gameSubState = gameSubState == "" ? "-----" : gameSubState;

        msg += format(
            "ROBOT:\n\tTeamID: %d\tPlayerID: %d\tNumberOfPlayers: %d\tRole: %s\tStartRole: %s\n\n",
            config->teamId,
            config->playerId,
            config->numOfPlayers,
            tree->getEntry<string>("player_role").c_str(),
            config->playerRole.c_str()
        );
        msg += format(
            "GAME:\n\tState: %s\tKickOffSide: %s\tisKickingOff: %s(%s)\n\tSubType: %s\tSubState: %s\tSubKickOffSide: %s\tisKickingOff: %s(%s)\n\tScore: %s\tJustScored: %s\n\tLiveCount: %d\tOppoLiveCount: %d\tPrimary: %s\n\n",
            gameState.c_str(),
            tree->getEntry<bool>("gc_is_kickoff_side") ? "YES" : "NO",
            data->isKickingOff ? "YES" : "NO",
            msecsSince(data->kickoffStartTime)/1000 > 100 ? "--" : to_string(msecsSince(data->kickoffStartTime)/1000).c_str(),
            gameSubType.c_str(),
            gameSubState.c_str(),
            tree->getEntry<bool>("gc_is_sub_state_kickoff_side") ? "YES" : "NO",
            data->isFreekickKickingOff ? "YES" : "NO",
            msecsSince(data->freekickKickoffStartTime)/1000 > 100 ? "--" : to_string(msecsSince(data->freekickKickoffStartTime)/1000).c_str(),
            format("%d:%d", data->score, data->oppoScore).c_str(),
            tree->getEntry<bool>("we_just_scored") ? "YES" : "NO",
            data->liveCount,
            data->oppoLiveCount,
            isPrimaryStriker() ? "YES" : "NO"
        );

        msg += getComLogString();

        msg += format(
            "DEBUG:\n\tcom: %s\tlogFile: %s\tlogTCP: %s\n\tvxFactor: %.2f\tyawOffset: %.2f\n\tControlState: %d\n\tTickTime: %.0fms",
            config->enableCom ? "YES" : "NO",
            config->rerunLogEnableFile ? "YES" : "NO",
            config->rerunLogEnableTCP ? "YES" : "NO",
            config->vxFactor,
            config->yawOffset,
            tree->getEntry<int>("control_state"),
            msecsSince(data->lastTick)
        );
        prtDebug(msg);
    }
    data->lastTick = get_clock()->now();
}

string Brain::getComLogString() {
    stringstream ss;
    int onFieldCnt = 0;
    int aliveCnt = 0;
    int selfIdx = config->playerId - 1;
    vector<int> onFieldIdxs = {};
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue;

        if (data->penalty[i] == PENALTY_NONE) {
            onFieldCnt += 1;
            onFieldIdxs.push_back(i);
        }

        if (data->tmStatus[i].isAlive) aliveCnt += 1;
    }
    ss << CYAN_CODE << "COM: " << "\n";
    ss << "Teammates: OnField: " << onFieldCnt << "[";
    for (int i = 0; i < onFieldIdxs.size(); i++) {
        int idx = onFieldIdxs[i];
        ss << " P" << idx + 1 << " ";
    }
    ss << "]";
    ss << "  Alive: " << aliveCnt << "  TMCMDID: " << data->tmCmdId << "  ReceivedDMD: " << data->tmReceivedCmd << "\n";

    // Self info
    ss << "Self\tCost: " << format("%.1f", data->tmMyCost) << "\tLead: ";
    if (data->tmImLead)
        ss << GREEN_CODE << "YES" << CYAN_CODE;
    else
        ss << RED_CODE << "NO" << CYAN_CODE;
    ss << "\tTeamRole: " << teamRoleCodeName(data->tmMyTeamRole);
    ss << "\tCaptain: [" << data->tmCaptainDecisionId << "] S=" << data->tmAssignedStrikerId << " P=" << data->tmAssignedSupporterId;
    ss << "    TMCMD: " << data->tmCmdId << format("\tCMD: [%d]%d", data->tmMyCmdId, data->tmMyCmd);
    ss << "\n";

    // Teammates info
    for (int i = 0; i < onFieldIdxs.size(); i++) {
        int idx = onFieldIdxs[i];
        auto status = data->tmStatus[idx];
        ss << "P" << idx + 1 << "[";
        if (status.isAlive)
            ss << GREEN_CODE << "鈽? << CYAN_CODE;
        else
            ss << RED_CODE << "鈽? << CYAN_CODE;
        ss << "]\tCost: " << format("%.1f", status.cost);
        ss << "\tLead: ";
        if (status.isLead)
            ss << GREEN_CODE << "YES" << CYAN_CODE;
        else
            ss << RED_CODE << "NO" << CYAN_CODE;
        ss << "\tState: " << robotStateCodeName(status.robotState);
        ss << "\tTeamRole: " << teamRoleCodeName(status.teamRole);
        ss << "\tCaptain: [" << status.captainDecisionId << "] S=" << status.assignedStrikerId << " P=" << status.assignedSupporterId;
        ss << "\tCMD: " << format("[%d]%d", status.cmdId, status.cmd);
        ss << "\tLag: " << format("%.0f", msecsSince(status.timeLastCom)) << "ms" << "\n";
    }
    ss << "\n";

    return ss.str();
}

bool Brain::isFreekickStartPlacing() {
    return (tree->getEntry<string>("gc_game_sub_state_type") == "FREE_KICK" && tree->getEntry<string>("gc_game_state") == "PLAY" && tree->getEntry<string>("gc_game_sub_state") == "GET_READY");
}

void Brain::playSoundForFun() {
    string soundPack = config->soundPack;
    if (config->soundEnable && soundPack != "espeak") {
        static string gcGameState_last;
        string gcGameState = tree->getEntry<string>("gc_game_state");

        static bool gameStarted = false;
        if (gcGameState == "PLAY") gameStarted = true;
        if (gcGameState == "READY")
        {
            if (!gameStarted) playSound(soundPack + "-ready", 2000);
            else if (tree->getEntry<bool>("we_just_scored")) playSound(soundPack + "-celebrate", 2000);
            else playSound(soundPack + "-regret", 2000);
        }

        if (gcGameState == "PLAY") {
            auto decision = tree->getEntry<string>("decision");
            if (decision == "chase") playSound(soundPack + "-chase", 5000);
            else if (decision == "adjust") playSound(soundPack + "-adjust", 2000);
            else if (decision == "kick") playSound(soundPack + "-kick", 2000);
        }
    }
}

// =============================================================================
//  璇峰皢浠ヤ笅浠ｇ爜杩藉姞鍒?src/brain/src/brain.cpp 鐨勬渶鏈熬 (鍦ㄦ渶鍚庣殑 } 涔嬪悗鎴栬€呬箣鍓嶅潎鍙紝纭繚鍦?namespace 澶栨垨绫诲畾涔夊)
// =============================================================================

/**
 * 璁＄畻褰撳墠鍚?dir 鏂瑰悜韪㈢悆鐨勪环鍊肩殑澶у皬.
 */
double Brain::kickValue(double dir)
{
    // 绠€鍗曠殑浠峰€艰瘎浼帮細濡傛灉鏄湞鍚戝鏂圭悆闂ㄦ柟鍚戯紝浠峰€奸珮
    // 杩欓噷鏄竴涓熀纭€瀹炵幇锛屼綘鍙互鏍规嵁绛栫暐闇€瑕佷慨鏀?
    auto fd = config->fieldDimensions;
    double goalDir = atan2(0.0 - data->robotPoseToField.y, fd.length/2.0 - data->robotPoseToField.x);
    double diff = fabs(toPInPI(dir - goalDir));

    // 瑙掑害鍋忓樊瓒婂皬锛屼环鍊艰秺楂樸€傝寖鍥?[0, 1]
    return max(0.0, 1.0 - diff / M_PI);
}

/**
 * 璁＄畻褰撳墠鐨勬€ヨ揩搴?
 * 0: 瀹夊叏, 1: 鏈夊▉鑳? 2: 鍗遍櫓
 */
double Brain::threatLevel()
{
    // 鍩虹瀹炵幇锛氬鏋滄湁鏁屼汉鍦ㄩ檮杩?2绫冲唴)锛屽垯璁や负鏈夊▉鑳?
    double minOpponentDist = 100.0;
    auto robots = data->getRobots();
    for(const auto& r : robots) {
        if(r.label == "Opponent") {
            double dist = r.range;
            if(dist < minOpponentDist) minOpponentDist = dist;
        }
    }

    if (minOpponentDist < 1.0) return 2.0;
    if (minOpponentDist < 2.5) return 1.0;
    return 0.0;
}

/**
 * 鍒ゆ柇鏄惁閫傚悎瀹氬悜韪㈢悆
 */
bool Brain::isAngleGoodForDirectionalKick(double goalPostMargin)
{
    // 澶嶇敤閫氱敤鐨勮搴﹀垽鏂€昏緫锛屾垨鑰呮坊鍔犵壒瀹氶€昏緫
    return isAngleGood(goalPostMargin, "kick");
}

/**
 * 妫€鏌ュ墠鏂规墖褰㈠尯鍩熸槸鍚︽湁闅滅鐗?
 */
bool Brain::isFrontRangeClear(double startAngle, double endAngle, double safeDist, double step)
{
    // 閬嶅巻瑙掑害鑼冨洿锛屾鏌ユ瘡涓搴︿笂鐨勬渶杩戦殰纰嶇墿璺濈
    for (double ang = startAngle; ang <= endAngle; ang += step) {
        if (distToObstacle(ang) < safeDist) {
            return false; // 鍙戠幇闅滅鐗╋紝涓嶇┖闂?
        }
    }
    return true; // 鍖哄煙鍐呮棤闅滅鐗?
}

/**
 * 鍙戝竷瑙嗚鏍″噯鍙傛暟
 */
/**
 * 鍙戝竷瑙嗚鏍″噯鍙傛暟
 */
void Brain::pubCalParamMsg(double pitch, double yaw, double z)
{
    if (pubCalParam) {
        vision_interface::msg::CalParam msg;
        msg.pitch_compensation = pitch;
        msg.yaw_compensation = yaw;
        msg.z_compensation = z;

        pubCalParam->publish(msg);
    }
}
