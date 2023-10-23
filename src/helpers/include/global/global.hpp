#ifndef GLOBAL_HPP_
#define GLOBAL_HPP_

#define DEG2RAD 0.017452925
#define RAD2DEG 57.295780
#define ODOTOCM 0.0131

enum BallStatus
{
    NOT_FOUND,
    FOUND,
    LOST,
    HELD
};

typedef struct Ball
{
    float pose_x;
    float pose_y;
    float pose_th;
    float dist;
} Ball_t;

typedef struct Robot
{
    float pose_x;
    float pose_y;
    float pose_th;
} Robot_t;

#endif