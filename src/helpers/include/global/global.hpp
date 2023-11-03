#ifndef GLOBAL_HPP_
#define GLOBAL_HPP_

#include <vector>
#include <cstdint>

#define DEG2RAD 0.017452925
#define RAD2DEG 57.295780
#define ODOTOCM 0.0131

//--->Field
#define FIELD_Y_0 0
#define FIELD_X_0 0
#define FIELD_Y_1 1200
#define FIELD_X_1 800

#define FIELD_Y_1_2 FIELD_Y_1 / 2
#define FIELD_X_1_2 FIELD_X_1 / 2

#define FIELD_Y_1_3 FIELD_Y_1 / 3
#define FIELD_X_1_3 FIELD_X_1 / 3

#define FIELD_Y_1_4 FIELD_Y_1 / 4
#define FIELD_X_1_4 FIELD_X_1 / 4

#define FIELD_Y_1_5 FIELD_Y_1 / 5
#define FIELD_X_1_5 FIELD_X_1 / 5

#define FIELD_Y_2_3 FIELD_Y_1 * 2 / 3
#define FIELD_X_2_3 FIELD_X_1 * 2 / 3

//--->Roles
#define GK 0
#define ATT 1
#define DEF 2
#define ASS 3
#define DEF2 4

//--->Roles
#define NO_AIM 0b0001
#define KICKER_POSITION_AIM 0b0010
#define THROTTLE_AIM 0b0100
#define OFFBALLWITHDRIBBLE 0b1000
#define ONLY_CHARGE 0b10000
#define ONLY_DISCHARGE 0b100000

enum BallStatus
{
    NOT_FOUND,
    FOUND,
    LOST,
    HELD
};

enum motion_flags
{
    normal = 0b00,
    invert = 0b01,
    normal_obstacle = 0b10,
    normal_obstacle_ball = 0b100,
};

enum obs_flags
{
    ignore_friends = 0b00001,
    near_ball = 0b00010,
    include_friends = 0b00100,
    nearest_pixel = 0b01000,
    include_ball = 0b100000,
};

typedef struct Ball
{
    float pose_x;
    float pose_y;
    float pose_th;
    float dist;
    uint8_t status;
} Ball_t;

typedef struct Robot
{
    float pose_x;
    float pose_y;
    float pose_th;
    float vel_x;
    float vel_y;
    float vel_th;
} Robot_t;

typedef struct MotorCommand
{
    float left;
    float right;
    float rear;
} MotorCommand_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
} pid_const_t;

typedef struct ControlCalc
{
    float x;
    float y;
    float th;
    float pose;
} ControlCalc_t;

typedef struct Obs
{
    uint8_t status;
    float angle;
    float distance;
    float pos_x;
    float pos_y;
} Obs_t;

typedef struct P3f
{
    float x;
    float y;
    float th;
} P3f_t;

typedef struct P2f
{
    float pose;
    float th;
} P2f_t;

//=-------Shared Data-------=//
//=-------------------------=//
typedef struct SharedDataRobot
{
    int16_t pos_x;                 // Pose_x of robot
    int16_t pos_y;                 // Pose_y of robot
    int16_t pos_theta;             // Pose_th of robot
    uint8_t state;                 // State of robot, one if it alive, and 0 if dead
    uint8_t role;                  // Assigned role of robot
    uint16_t robot_cond;           // Another robot's state
    int16_t obs_x[144];            // x_obstacle of robot
    int16_t obs_y[144];            // y_obstacle of robot
    uint8_t obs_total;             // Total obstacle that robot can detect
    uint16_t obs_r_final[144];     // Obstacle magnitude
    uint16_t obs_angle_final[144]; // Obstacle direction
    uint8_t target_on_field;       // Target index passing
    uint8_t is_free;               // Not used
    uint8_t obs_r_final_dumped[144];
    uint8_t obs_angle_final_dumped[144];
    uint8_t total_obs_dumped;
    int16_t passing_point_x;
    int16_t passing_point_y;
} SharedDataRobot_t;

MotorCommand_t motor_command;
Robot_t robot;
SharedDataRobot_t robot_data[6];
Ball ball;

pid_const_t to_point_position;
pid_const_t to_point_angle;
pid_const_t around_point_position;
pid_const_t around_point_angle;
pid_const_t around_point_arc;
pid_const_t stm_pid;

P3f_t target_point;
P2f_t raw_velocity;

float attr_rad = 0;                        // Attractive radius
float repl_rad = 0;                        // Repulsive radius
std::vector<uint16_t> dumped_obs_on_field; // This is a vector of obstacle on field that has been merged
uint8_t dumped_obs_total;                  // This is a total of obstacle on field that has been merged
std::vector<std::uint16_t> obs_on_field;

// Game States
uint8_t robot_base_action; // This is base action of robot, the state machine will be multplier by this

void AssignRobotTarget(float _x, float _y, float _th)
{
    target_point.x = _x;
    target_point.y = _y;
    target_point.th = _th;
}

void AssignRawVelocity(float _vpos, float _vth)
{
    raw_velocity.pose = _vpos;
    raw_velocity.th = _vth;
}
#endif