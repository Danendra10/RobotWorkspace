#ifndef MOTION_HPP_
#define MOTION_HPP_

#pragma once

#include <math/potential_field/vector_attractive.h>
#include <math/potential_field/vector_repulsive.h>
#include <math/simple_math.hpp>
#include <logger/logger.hpp>
#include <global/global.hpp>
#include <pid/pid.hpp>

using namespace std;

ControlCalc_t error;
ControlCalc_t output;

extern MotorCommand motor_command;
extern Robot robot;
extern SharedDataRobot_t robot_data[6];
extern Ball ball;

typedef struct AvoidObs
{
    uint8_t status;
    float angle;
    float distance;
    float vx;
    float vy;
} AvoidObs_t;

void ManualMotion(int16_t _vx, int16_t _vy, int16_t _vth, int8_t acceleration = 3, Robot_t *ret = &robot);
void ManualMotionF(float _vx, float _vy, float _vth, int8_t acceleration = 3, Robot_t *ret = &robot);
void ManualMotionPosition(int16_t _vx, int16_t _vy, int16_t _vth, Robot_t *ret = &robot);
void ManualMotionPositionF(float _vx, float _vy, float _vth, Robot_t *ret = &robot);
bool MotionToPoint(float target_x, float target_y, float target_th, uint16_t vel_position, uint16_t vel_th, int16_t flags = 0, Robot *ret = &robot);
bool PotentialMotion(float target_x, float target_y, float target_th, float vel_position, float vel_th, int16_t flags = 0, Robot *ret = &robot);

AvoidObs_t ObstacleAvoidance(float vx_input, float vy_input, float angle_tolerance = 25, float dist = 100, uint8_t flags = nearest_pixel);
Obs_t ObstacleChecking(float theta, float tolerance = 25, float dist = 100, uint8_t flags = nearest_pixel);
#endif // MOTION_HPP_