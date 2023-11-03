#ifndef SIMPLE_MATH_HPP_
#define SIMPLE_MATH_HPP_

#include <cmath>
#include "global/global.hpp"

float Pythagoras(float _x1, float _y1, float _x2, float _y2);
float RobotAngletoPoint(int16_t x, int16_t y);
float RobotAngletoBall();
float RobotAngletoEnemyGoal();
float RobotAngletoOwnGoal();
float RobotAngletoRobot(uint8_t robot_id);
#endif