#include "global/global.hpp"

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