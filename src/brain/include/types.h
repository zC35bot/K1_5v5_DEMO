/**
 * @file type.h
 * @brief 定义 brain 项目中用到的 struct 及 enum
 */
#pragma once

#include <string>
#include <vector>
#include <numeric>
#include <iterator>
#include <limits>
#include <rclcpp/rclcpp.hpp>

using namespace std;

/* ------------------ Struct ------------------------*/

// 球场尺寸信息, 所有平行于底线的是长度, 所有垂直于底线的是宽度, 与线的长度无关
struct FieldDimensions
{
    double length;            // 球场长度
    double width;             // 球场宽度
    double penaltyDist;       // 罚球点距离底线的直线距离
    double goalWidth;         // 球门的宽度
    double circleRadius;      // 中圈的半径
    double penaltyAreaLength; // 禁区的长
    double penaltyAreaWidth;  // 禁区的宽
    double goalAreaLength;    // 球门区的长
    double goalAreaWidth;     // 球门区的宽
                              // 注意: 禁区比球门区大；禁区和球门区的长与宽实际上要小。这个命名是为了与比赛规则相统一。
};
const FieldDimensions FD_KIDSIZE{9, 6, 1.5, 2.6, 0.75, 2, 5, 1, 3};
// const FieldDimensions FD_ADULTSIZE{14, 9, 2.1, 2.6, 1.5, 3, 6, 1, 4};
const FieldDimensions FD_ADULTSIZE{14.16, 9.22, 2.242, 2.6, 1.54, 3.24, 6.192, 1.345, 4};
// const FieldDimensions FD_ROBOLEAGUE{22, 14, 3.6, 2.6, 2, 2.25, 6.9, 0.75, 3.9};
// const FieldDimensions FD_ROBOLEAGUE{22, 14, 3.5, 2.6, 2, 5, 8, 2, 5};
const FieldDimensions FD_ROBOLEAGUE{22.003, 14.126, 3.635, 2.6, 1.99, 5.221, 8.121, 2.307, 5.083};

// Pose2D, 记录平面上的一个点以及朝向
struct Pose2D
{
    double x = 0;
    double y = 0;
    double theta = 0; // rad, 从 x 轴正方向开始, 逆时针为正
};

// Point, 记录一个三维点
struct Point
{
    double x = 0;
    double y = 0;
    double z = 0;
};

// Point2D, 记录一个二维点
struct Point2D
{
    double x = 0;
    double y = 0;
};

// BoundingBox
struct BoundingBox
{
    double xmin = 0, xmax = 0, ymin = 0, ymax = 0; // 类内默认初始化, 避免首次检测前读到未初始化值
};

// GameObject, 用于存储比赛中的重要实体信息，如 Ball, Goalpost 等。相比于 /detect 消息中的 detection::DetectedObject，它的信息更为丰富。
struct GameObject
{
    // --- 从 /detect 消息中获得 ---
    string label;                // 物体被识别为什么
    string color;                // 物体的颜色. 只用于 Opponent, 用于识别敌我
    BoundingBox boundingBox;     // 物体在摄像头中的识别框, 左上角为 0 点, 向右为 x, 向下为 y
    Point2D precisePixelPoint;   // 物体的精确像素点位置, 仅地面标志点有这一数据
    double confidence = 0;       // 识别的置信度, 对 obstacle 来说, 是大于 0 的数字, 代表障碍网格中高于阈值的点数 (默认初始化)
    Point posToRobot;            // 物体在机器人本体坐标系的的位置, 位置为 2D, 忽略 z 值.

    // --- 在 processDetectedObject 函数中计算获得 ---
    Point posToField;                                 // 物体在物体场坐标系的的位置, 位置为 2D, 忽略 z 值. x 向前, y 向左.
    double range = 0;                                 // 物体距离机器人中心在物体场平面上的投影点的直线距离 (默认初始化)
    double pitchToRobot = 0, yawToRobot = 0;          // 物体相对于机器人正前方的 pitch 和 yaw, 单位 rad, 向下和向左为正 (默认初始化)
    rclcpp::Time timePoint;                           // 物体被检测到的时间

    // --- 通过各类不同对象的特殊处理获得, 只有部分对象有这些值 ---
    int id = 0;    // 识别出来的 id (默认初始化)
    string name;  // human readable id
    double idConfidence = 0; // id 识别的置信度 [0, 1] 区间, (默认初始化)
    int positionConfidence = 0; // 视觉侧附带的位置来源/模式编码
    string info; // 用于存储额外的信息
};

// Line, 用 x0, y0, x1, y1 表示任意坐标系下的一条直线.
struct Line {
    double x0, y0, x1, y1;
};

struct FieldLineRef {
    std::string name;
    double x0, y0, x1, y1; // 两端点，单位：米
    bool isVertical;
};

// enum LineID {
//     LT, // left touch line
//     RT, // right touch line
//     M, // middle line
//     SB, // self bottom (goal line)
//     OB, // opposite bottom (goal line)
//     SGL, // self goal area left
//     SGH, // self goal area horizontal
//     SGR, // self goal area right
//     OGL, // opposite goal area left
//     OGH, // opposite goal area horizontal
//     OGR, // opposite goal area right
//     SPL, // self penalty area left
//     SPH, // self penalty area horizontal
//     SPR, // self penalty area right
//     OPL, // opposite penalty area left
//     OPH, // opposite penalty area horizontal
//     OPR // opoosite penalty area right
// };

enum class LineHalf {
    NA,
    Self,
    Opponent
};

enum class LineDir {
    NA,
    Horizontal,
    Vertical
};

enum class LineSide {
    NA,
    Left,
    Right
};

enum class LineType {
    NA,
    TouchLine,
    MiddleLine,
    GoalLine,
    GoalArea,
    PenaltyArea
};

// FieldLine 用于存储场地内的一条标线信息
struct FieldLine {
    Line posToField;
    Line posToRobot;
    Line posOnCam;
    rclcpp::Time timePoint;                           
    LineDir dir;                                       
    LineHalf half;                                      
    LineSide side;                                      
    LineType type;
    double confidence;                                      
};

// 用于存储地图中地面标志点的理论位置信息
struct MapMarking {
    double x;
    double y;
    string type; // TCross | LCross | XCross | PenaltyPoint
    string name; // name 的规则为四位string, 如 LSCL, 第一位: L|T|X|P 代表 type, 第二位: S|M|O 代表半场, 第三位: L|M|R 代表左右, 第四位: B(boundary)|P(penalty area)|G(gaol area)|C(circle) 代表点所在的方形(或圆形)
};

// VisionBox 用于视野的范围
struct VisionBox {
    vector<double> posToField;
    vector<double> posToRobot;
    rclcpp::Time timePoint;                           // 物体被检测到的时间
};

struct TimestampedData {
    vector<double> data;
    rclcpp::Time timePoint;
};

// 起身
struct RobotRecoveryStateData {
    uint8_t state; // IS_READY = 0, IS_FALLING = 1, HAS_FALLEN = 2, IS_GETTING_UP = 3,  
    uint8_t is_recovery_available; // 1 for available, 0 for not available
    uint8_t current_planner_index;
};

enum RobotStateCode {
    ROBOT_STATE_UNKNOWN = 0,
    ROBOT_STATE_WAITING_START = 1,
    ROBOT_STATE_MANUAL = 2,
    ROBOT_STATE_ENTERING_FIELD = 3,
    ROBOT_STATE_PENALIZED = 4,
    ROBOT_STATE_WAIT_OPPONENT_KICKOFF = 5,
    ROBOT_STATE_GOALIE_GUARD = 6,
    ROBOT_STATE_FIND_BALL = 7,
    ROBOT_STATE_BALL_FOUND = 8,
    ROBOT_STATE_CHASE_BALL = 9,
    ROBOT_STATE_ADJUST_BALL = 10,
    ROBOT_STATE_KICK_BALL = 11,
    ROBOT_STATE_CROSS_BALL = 12,
    ROBOT_STATE_VISUAL_KICK = 13,
    ROBOT_STATE_ASSIST = 14,
    ROBOT_STATE_RETREAT = 15,
    ROBOT_STATE_INTERCEPT = 16,
    ROBOT_STATE_ONE_TWO_GO = 17,
};

inline string robotStateCodeName(int code) {
    switch (code) {
    case ROBOT_STATE_WAITING_START: return "waiting_start";
    case ROBOT_STATE_MANUAL: return "manual";
    case ROBOT_STATE_ENTERING_FIELD: return "entering_field";
    case ROBOT_STATE_PENALIZED: return "penalized";
    case ROBOT_STATE_WAIT_OPPONENT_KICKOFF: return "wait_opponent_kickoff";
    case ROBOT_STATE_GOALIE_GUARD: return "goalie_guard";
    case ROBOT_STATE_FIND_BALL: return "find_ball";
    case ROBOT_STATE_BALL_FOUND: return "ball_found";
    case ROBOT_STATE_CHASE_BALL: return "chase_ball";
    case ROBOT_STATE_ADJUST_BALL: return "adjust_ball";
    case ROBOT_STATE_KICK_BALL: return "kick_ball";
    case ROBOT_STATE_CROSS_BALL: return "cross_ball";
    case ROBOT_STATE_VISUAL_KICK: return "visual_kick";
    case ROBOT_STATE_ASSIST: return "assist";
    case ROBOT_STATE_RETREAT: return "retreat";
    case ROBOT_STATE_INTERCEPT: return "intercept";
    case ROBOT_STATE_ONE_TWO_GO: return "one_two_go";
    default: return "unknown";
    }
}

enum TeamRoleCode {
    TEAM_ROLE_UNKNOWN = 0,
    TEAM_ROLE_GOALKEEPER = 1,
    TEAM_ROLE_STRIKER = 2,
    TEAM_ROLE_SUPPORTER = 3,
};

inline string teamRoleCodeName(int code) {
    switch (code) {
    case TEAM_ROLE_GOALKEEPER: return "goalkeeper";
    case TEAM_ROLE_STRIKER: return "striker";
    case TEAM_ROLE_SUPPORTER: return "supporter";
    default: return "unknown";
    }
}

enum PassStateCode {
    PASS_STATE_IDLE = 0,
    PASS_STATE_EVALUATE_PASS = 1,
    PASS_STATE_PASSING = 2,
    PASS_STATE_WAIT_RECEIVE_SWITCH = 3,
    PASS_STATE_EXIT = 4,
};

inline string passStateCodeName(int code) {
    switch (code) {
    case PASS_STATE_EVALUATE_PASS: return "evaluate";
    case PASS_STATE_PASSING: return "passing";
    case PASS_STATE_WAIT_RECEIVE_SWITCH: return "wait_receive_switch";
    case PASS_STATE_EXIT: return "exit";
    default: return "idle";
    }
}

enum OneTwoStateCode {
    ONE_TWO_STATE_IDLE = 0,
    ONE_TWO_STATE_ARMED = 1,
    ONE_TWO_STATE_PASS_AND_GO = 2,
    ONE_TWO_STATE_ONE_TOUCH_RETURN = 3,
    ONE_TWO_STATE_WAIT_REACQUIRE = 4,
    ONE_TWO_STATE_TIMEOUT_EXIT = 5,
};

inline string oneTwoStateCodeName(int code) {
    switch (code) {
    case ONE_TWO_STATE_ARMED: return "armed";
    case ONE_TWO_STATE_PASS_AND_GO: return "pass_and_go";
    case ONE_TWO_STATE_ONE_TOUCH_RETURN: return "one_touch_return";
    case ONE_TWO_STATE_WAIT_REACQUIRE: return "wait_reacquire";
    case ONE_TWO_STATE_TIMEOUT_EXIT: return "timeout_exit";
    default: return "idle";
    }
}

// 用于存储队友间通讯
struct TMStatus {
    string role = "not initialized"; // triker, goal_keeper
    int teamRole = TEAM_ROLE_UNKNOWN; // 战术角色: goalkeeper / striker / supporter
    bool isAlive = false; // 是否在场上, 且没有在罚时中, 且通讯没有丢失
    bool isFallen = false; // 队友是否处于倒地状态
    bool ballDetected = false;
    bool ballLocationKnown = false;
    double ballConfidence = 0.;
    double ballRange = 0.;
    double cost = 0.; // 计算从当前状态到能踢到球的成本
    bool isLead = true; // 是否在控球状态
    Point ballPosToField;
    Pose2D robotPoseToField;
    double kickDir = 0.; // 预计的踢球方向
    double thetaRb = 0.; // 机器人到球的角度, field 坐标系
    int robotState = ROBOT_STATE_UNKNOWN; // 队友当前高层状态编码, 与 robotStateCodeName 对应
    int cmd = 0; // 最后一次发出的指令 
    int cmdId = 0; // 最后一次发出的指令 ID
    int assignedStrikerId = 0; // 守门员当前判定的主攻球员 id
    int assignedSupporterId = 0; // 守门员当前判定的辅助球员 id
    int captainDecisionId = 0; // 守门员角色分配决策序号
    bool passInitiator = false; // 是否是这次常规传球/二过一的发起者
    int passState = PASS_STATE_IDLE; // 常规传球状态机
    int passPartnerPlayerId = 0; // 本次配合的对端球员 id
    int passSequenceId = 0; // 本次配合序列号
    bool passReceiveReady = false; // 接球队员是否已到位
    bool passTakeoverAck = false; // 接球队员是否已接管
    bool passOneTwoIntent = false; // 是否带有二过一意图
    Point passTargetPosToField; // 传球落点 / 接球点
    int oneTwoState = ONE_TWO_STATE_IDLE; // 二过一状态机
    Point oneTwoReturnTargetPosToField; // 二过一回做目标点
    rclcpp::Time timeLastCom; // 最后一次通讯时间
};


enum class RobotRecoveryState {
    IS_READY = 0,
    IS_FALLING = 1,
    HAS_FALLEN = 2,
    IS_GETTING_UP = 3
};
