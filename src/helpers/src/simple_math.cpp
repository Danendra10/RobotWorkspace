#include <math/simple_math.hpp>
float Pythagoras(float _x1, float _y1, float _x2, float _y2)
{
    // This avoids the need to call std::pow twice, making the code faster and more efficient.
    float dx = _x2 - _x1;
    float dy = _y2 - _y1;
    return std::sqrt(dx * dx + dy * dy);
}

float RobotAngletoPoint(int16_t x, int16_t y)
{
    return atan2(y - robot.pose_y, x - robot.pose_x) * RAD2DEG;
}

float RobotAngletoBall()
{
    return RobotAngletoPoint(ball.pose_x, ball.pose_y);
}

float RobotAngletoEnemyGoal()
{
    return RobotAngletoPoint(FIELD_X_1_2, FIELD_Y_1);
}

float RobotAngletoOwnGoal()
{
    return RobotAngletoPoint(FIELD_X_1_2, FIELD_Y_0);
}

float RobotAngletoRobot(uint8_t robot_id)
{
    return RobotAngletoPoint(robot_data[robot_id].pos_x, robot_data[robot_id].pos_y);
}