#include <motion/motion.hpp>

void ManualMotion(int16_t _vx, int16_t _vy, int16_t _vth, int8_t acceleration, Robot *ret)
{
    static float v_buffer[2];

    // Convert velocity to cm/s
    float vx_cvt = _vx * 1.20;
    float vy_cvt = _vy * 0.60;

    // Calculate the change in velocity
    float delta_v[2] = {vx_cvt - v_buffer[0], vy_cvt - v_buffer[1]};

    // Limit the maximum distance
    float r = std::sqrt(delta_v[0] * delta_v[0] + delta_v[1] * delta_v[1]);
    if (r > 3.2)
        r = 3.2;

    // Calculate the new velocity
    float theta = std::atan2(delta_v[1], delta_v[0]);
    v_buffer[0] += r * std::cos(theta);
    v_buffer[1] += r * std::sin(theta);

    // Assign the new velocity to the robot object
    ret->vel_x = v_buffer[0];
    ret->vel_y = v_buffer[1];
    ret->vel_th = _vth;
}

void ManualMotionF(float _vx, float _vy, float _vth, int8_t acceleration, Robot *ret)
{
    static float v_buffer[2];
    static bool initialized = false;

    if (!initialized)
    {
        v_buffer[0] = _vx;
        v_buffer[1] = _vy;
        initialized = true;
    }

    float delta_v[2];
    delta_v[0] = _vx - v_buffer[0];
    delta_v[1] = _vy - v_buffer[1];

    float r = std::sqrt(delta_v[0] * delta_v[0] + delta_v[1] * delta_v[1]);
    float theta = std::atan2(delta_v[1], delta_v[0]);

    r = std::min(r, (float)acceleration);

    v_buffer[0] += r * std::cos(theta);
    v_buffer[1] += r * std::sin(theta);

    ret->vel_x = v_buffer[0];
    ret->vel_y = v_buffer[1];
    ret->vel_th = _vth;
}

void ManualMotionPosition(int16_t _vx, int16_t _vy, int16_t _vth, Robot *ret)
{
    float sin_y = std::sin(robot.pose_y * DEG2RAD); // avoid do a redundant calculation
    float cos_y = std::cos(robot.pose_y * DEG2RAD);

    float vector_output[2];
    vector_output[0] = _vx * sin_y - _vy * cos_y;
    vector_output[1] = _vx * cos_y + _vy * sin_y;

    ManualMotion(vector_output[0], vector_output[1], _vth, 3, ret);
}

void ManualMotionPositionF(float _vx, float _vy, float _vth, Robot *ret)
{
    float sin_y = std::sin(robot.pose_y * DEG2RAD);
    float cos_y = std::cos(robot.pose_y * DEG2RAD);

    float vector_output[2];
    vector_output[0] = _vx * sin_y - _vy * cos_y;
    vector_output[1] = _vx * cos_y + _vy * sin_y;

    ManualMotionF(vector_output[0], vector_output[1], _vth, 3, ret);
}

bool MotionToPoint(float target_x, float target_y, float target_th, uint16_t vel_position, uint16_t vel_th, int16_t flags, Robot *ret)
{
    static PID_t position_pid;
    static PID_t angles_pid;

    PIDInit(&position_pid, to_point_position.kp, to_point_position.ki, to_point_position.kd);
    PIDInit(&angles_pid, to_point_angle.kp, to_point_angle.ki, to_point_angle.kd);

    // Calculate error position
    float error_x = target_x - robot.pose_x;
    float error_y = target_y - robot.pose_y;
    float error_pose = sqrt(error_x * error_x + error_y * error_y);

    // Calculate error theta
    float error_th = target_th - robot.pose_th;
    while (error_th < -180)
        error_th += 360;
    while (error_th > 180)
        error_th -= 360;

    // Calculate pose output
    float pose_output = PIDCalculate(&position_pid, error_pose, vel_position);
    float output_x_buffer = pose_output * cos(atan2(error_y, error_x));
    float output_y_buffer = pose_output * sin(atan2(error_y, error_x));

    // Calculate angle output
    float angle_output = PIDCalculate(&angles_pid, error_th, vel_th);

    // Calculate obstacle avoidance outputs
    if ((flags & normal_obstacle) == normal_obstacle)
    {
        output_x_buffer = ObstacleAvoidance(output_x_buffer, output_y_buffer, 15, 60, nearest_pixel).vx;
        output_y_buffer = ObstacleAvoidance(output_x_buffer, output_y_buffer, 15, 60, nearest_pixel).vy;
    }
    else if ((flags & normal_obstacle_ball) == normal_obstacle_ball)
    {
        output_x_buffer = ObstacleAvoidance(output_x_buffer, output_y_buffer, 15, 60, nearest_pixel | include_ball).vx;
        output_y_buffer = ObstacleAvoidance(output_x_buffer, output_y_buffer, 15, 60, nearest_pixel | include_ball).vy;
    }

    // Update output
    float output_x = output_x_buffer;
    float output_y = output_y_buffer;
    float output_th = angle_output;

    // Perform manual motion
    ManualMotionPosition(output_x, output_y, output_th, ret);

    // Check if error threshold reached
    if (fabs(error_th) < 5 && fabs(error_pose) < 3)
    {
        ManualMotion(0, 0, 0);
        return true;
    }
    else
        return false;
}

Obs_t ObstacleChecking(float theta, float tolerance, float dist, uint8_t flags)
{
    Obs_t obs_ret; // init new obj
    int16_t init_index;
    int16_t final_index;
    static uint8_t obs_start_idx;
    static uint8_t obs_end_idx;
    uint8_t obs_cnt_idx; // Counter
    static uint8_t prev_obs_state;
    static uint8_t obs_state;
    static float obs_angle;
    static float obs_x_field_buffer[144];
    static float obs_y_field_buffer[144];
    uint8_t obs_cnt;
    uint8_t idx;
    std::vector<float> obs_dist_buffer;
    std::vector<uint8_t> obs_idx_buffer;
    float obs_x_buffer;
    float obs_y_buffer;

    init_index = (theta - tolerance) * 0.4;
    final_index = (theta + tolerance) * 0.4;

    // Reset vars
    prev_obs_state = 0;
    obs_cnt_idx = 0;
    obs_cnt = 0;
    obs_ret.distance = dist;
    obs_ret.status = 0;
    obs_ret.angle = 0;

    for (int16_t i = init_index; i <= final_index; i++)
    {
        idx = i + (i < 0) * 144 - (i >= 144) * 144;

        obs_state = (obs_on_field[idx] <= dist && i != final_index && obs_on_field[idx] > 0); // get current obs state

        if (prev_obs_state == 0 && obs_state == 1)
        {
            obs_start_idx = idx; // get start index

            // record current val
            obs_x_field_buffer[obs_cnt_idx] = robot.pose_x + obs_on_field[idx] * cos(idx * 2.5 * DEG2RAD);
            obs_y_field_buffer[obs_cnt_idx] = robot.pose_y + obs_on_field[idx] * sin(idx * 2.5 * DEG2RAD);

            obs_cnt_idx++;
        }
        else if (prev_obs_state == 1 && obs_state == 1 && obs_cnt_idx > 0)
        {
            // record current val
            obs_x_field_buffer[obs_cnt_idx] = robot.pose_x + obs_on_field[idx] * cos(idx * 2.5 * DEG2RAD);
            obs_y_field_buffer[obs_cnt_idx] = robot.pose_y + obs_on_field[idx] * sin(idx * 2.5 * DEG2RAD);

            obs_cnt_idx++;
        }
        else if (prev_obs_state == 1 && obs_state == 0 && obs_cnt_idx > 0)
        {
            obs_end_idx = idx - 1;                            // get end idx
            obs_angle = (obs_end_idx + obs_start_idx) * 1.25; // Get angle (degrees)

            // Get mean
            for (uint8_t j = 0; j < obs_cnt_idx; j++)
            {
                obs_x_buffer += obs_x_field_buffer[j];
                obs_y_buffer += obs_y_field_buffer[j];
            }
            obs_x_buffer /= obs_cnt_idx;
            obs_y_buffer /= obs_cnt_idx;
            // Get center of obs
            obs_x_buffer += 25 * cos(obs_angle * DEG2RAD);
            obs_y_buffer += 25 * sin(obs_angle * DEG2RAD);

            // Save to buffer
            obs_dist_buffer.push_back(Pythagoras(robot.pose_x, robot.pose_y, obs_x_buffer, obs_y_buffer));
            obs_idx_buffer.push_back((obs_end_idx + obs_start_idx) * 0.5);

            obs_cnt_idx = 0; // Reset counter
            obs_cnt++;       // Update total obs
        }

        prev_obs_state = obs_state;

        // nearest_pixel
        if ((flags & nearest_pixel) == nearest_pixel)
        {
            if (obs_on_field[idx] < obs_ret.distance && obs_on_field[idx] > 0)
            {
                obs_ret.angle = idx * 2.5;
                obs_ret.distance = (float)obs_on_field[idx];
                obs_ret.pos_x = robot.pose_x + obs_on_field[idx] * cos(idx * 2.5 * DEG2RAD);
                obs_ret.pos_y = robot.pose_y + obs_on_field[idx] * sin(idx * 2.5 * DEG2RAD);
                obs_cnt++;
            }
        }
    }

    // Add friends to obs buffer
    if ((flags & include_friends) == include_friends)
    {
        for (uint8_t j = 1; j <= 5; j++)
        {
            if (robot_data[j].state)
            {
                float distance_buffer = Pythagoras(robot.pose_x, robot.pose_y, robot_data[j].pos_x, robot_data[j].pos_y);
                obs_dist_buffer.push_back(distance_buffer);
                int16_t angle_to_friend = (int)(RobotAngletoPoint(robot_data[j].pos_x, robot_data[j].pos_y));
                obs_idx_buffer.push_back(angle_to_friend * 0.4 + (angle_to_friend * 0.4 < 0) * 144);
                obs_cnt++; // Update total obs

                if ((flags & nearest_pixel) == nearest_pixel &&
                    fabs(theta - angle_to_friend) < tolerance &&
                    distance_buffer < obs_ret.distance)
                {
                    obs_ret.distance = distance_buffer;
                    obs_ret.pos_x = distance_buffer * cosf(angle_to_friend * DEG2RAD);
                    obs_ret.pos_y = distance_buffer * sinf(angle_to_friend * DEG2RAD);
                    obs_ret.angle = angle_to_friend;
                    obs_ret.status = (obs_cnt > 0);
                    // return obs_ret;
                }
            }
        }
    }
    // Add ball to obs buffer
    if ((flags & include_ball) == include_ball && ball.status == 1)
    {
        obs_cnt++; // Update total obs
        obs_dist_buffer.push_back(ball.dist);
        obs_idx_buffer.push_back(ball.pose_th * 0.4 + (ball.pose_th * 0.4 < 0) * 144);

        if ((flags & nearest_pixel) == nearest_pixel &&
            fabs(theta - ball.pose_th) < tolerance &&
            ball.dist < obs_ret.distance)
        {
            obs_ret.distance = ball.dist;
            obs_ret.pos_x = ball.pose_x;
            obs_ret.pos_y = ball.pose_y;
            obs_ret.angle = ball.pose_th;
            obs_ret.status = (obs_cnt > 0);
            // return obs_ret;
        }
    }

    // printf("wow3 %d\n", idx);

    // post processing, get obs data depend on flags
    if ((flags & nearest_pixel) == nearest_pixel)
    {
        // printf("end %d\n", idx);
        obs_ret.status = (obs_cnt > 0);
        return obs_ret;
    }

    // post processing, get obs data depend on flags
    if (obs_cnt > 1)
    {
        float obs_dist_final = obs_dist_buffer[0];
        float obs_idx_final = obs_idx_buffer[0];

        // Process
        if (((flags & ignore_friends) == ignore_friends) && ((flags & near_ball) == near_ball))
        {
            uint8_t obs_valid = 0; // always init with 0 value

            // iterate through buffer
            for (uint8_t i = 0; i < obs_dist_buffer.size(); i++)
            {
                static float prev_dist = 9999;
                int8_t status_obs_valid = 1;
                float obs_x = robot.pose_x + obs_dist_buffer[i] * cos(obs_idx_buffer[i] * 2.5 * DEG2RAD);
                float obs_y = robot.pose_y + obs_dist_buffer[i] * sin(obs_idx_buffer[i] * 2.5 * DEG2RAD);

                // Ignore friends
                for (uint8_t j = 1; j <= 5; j++)
                {
                    if (robot_data[j].state)
                    {
                        // Obs invalid if it close to friend
                        status_obs_valid = !(Pythagoras(obs_x, obs_y, robot_data[j].pos_x, robot_data[j].pos_y) < 75);
                    }
                }

                // Get obs closest to ball
                if (status_obs_valid && Pythagoras(obs_x, obs_y, ball.pose_x, ball.pose_y) < prev_dist)
                {
                    obs_dist_final = obs_dist_buffer[i];
                    obs_idx_final = obs_idx_buffer[i];
                    obs_valid++;
                }
                prev_dist = Pythagoras(obs_x, obs_y, ball.pose_x, ball.pose_y);
            }
            obs_ret.status = (obs_valid > 0); // set status depend on obs_valid
        }
        else if ((flags & near_ball) == near_ball)
        {
            // iterate through buffer
            for (uint8_t i = 0; i < obs_dist_buffer.size(); i++)
            {
                static float prev_dist = 9999;
                float obs_x = robot.pose_x + obs_dist_buffer[i] * cos(obs_idx_buffer[i] * 2.5 * DEG2RAD);
                float obs_y = robot.pose_y + obs_dist_buffer[i] * sin(obs_idx_buffer[i] * 2.5 * DEG2RAD);

                // Get obs closest to ball
                if (Pythagoras(obs_x, obs_y, ball.pose_x, ball.pose_y) < prev_dist)
                {
                    obs_dist_final = obs_dist_buffer[i];
                    obs_idx_final = obs_idx_buffer[i];
                }
                prev_dist = Pythagoras(obs_x, obs_y, ball.pose_x, ball.pose_y);
            }
        }
        else if ((flags & ignore_friends) == ignore_friends)
        {
            uint8_t obs_valid = 0; // always init with 0 value

            // iterate through buffer
            for (uint8_t i = 0; i < obs_dist_buffer.size(); i++)
            {
                int8_t status_obs_valid = 1;
                float obs_x = robot.pose_x + obs_dist_buffer[i] * cos(obs_idx_buffer[i] * 2.5 * DEG2RAD);
                float obs_y = robot.pose_y + obs_dist_buffer[i] * sin(obs_idx_buffer[i] * 2.5 * DEG2RAD);

                // Ignore friends
                for (uint8_t j = 1; j <= 5; j++)
                {
                    if (robot_data[j].state)
                    {
                        // Obs invalid if it close to friend
                        status_obs_valid = !(Pythagoras(obs_x, obs_y, robot_data[j].pos_x, robot_data[j].pos_y) < 75);
                    }
                }

                // Find obs that closest to robot
                if (obs_dist_buffer[i] < obs_dist_final && status_obs_valid)
                {
                    obs_dist_final = obs_dist_buffer[i];
                    obs_idx_final = obs_idx_buffer[i];
                    obs_valid++;
                }
            }
            obs_ret.status = (obs_valid > 0); // set status depend on obs_valid
        }
        else
        {
            // iterate through buffer
            for (uint8_t i = 0; i < obs_dist_buffer.size(); i++)
            {
                if (obs_dist_buffer[i] < obs_dist_final)
                {
                    obs_dist_final = obs_dist_buffer[i];
                    obs_idx_final = obs_idx_buffer[i];
                }
            }
        }

        // Set final value
        obs_ret.pos_x = robot.pose_x + obs_dist_final * cos(obs_idx_final * 2.5 * DEG2RAD);
        obs_ret.pos_y = robot.pose_y + obs_dist_final * sin(obs_idx_final * 2.5 * DEG2RAD);
        obs_ret.angle = obs_idx_final * 2.5;
        obs_ret.distance = obs_dist_final;
    }
    else if (obs_cnt == 1)
    {
        // Final value from index 0
        obs_ret.pos_x = robot.pose_x + obs_dist_buffer[0] * cos(obs_idx_buffer[0] * 2.5 * DEG2RAD);
        obs_ret.pos_y = robot.pose_y + obs_dist_buffer[0] * sin(obs_idx_buffer[0] * 2.5 * DEG2RAD);
        obs_ret.angle = obs_idx_buffer[0] * 2.5;
        obs_ret.distance = obs_dist_buffer[0];
    }
    else
    {
        printf("else\n");
        obs_ret.pos_x = 9999;
        obs_ret.pos_y = 9999;
        obs_ret.angle = 0;
        obs_ret.distance = 9999;
    }

    // if obs_cnt > 0, status = 1. else status = 0
    obs_ret.status = (obs_cnt > 0);

    // printf("halooo %d %.02f %.02f %.02f %.02f\n", obs_ret.status, obs_ret.distance, obs_ret.angle, obs_ret.pos_x, obs_ret.pos_y);

    return obs_ret;
}

AvoidObs_t ObstacleAvoidance(float vx_input, float vy_input, float angle_tolerance, float dist, uint8_t flags)
{
    float angle_target = atan2(vy_input, vx_input) * RAD2DEG;
    float translation_vel = sqrt(vx_input * vx_input + vy_input * vy_input);

    Obs_t obs = ObstacleChecking(angle_target, angle_tolerance, dist, flags);
    AvoidObs_t ret;

    if (obs.status)
    {
        float delta_angle = angle_target - obs.angle; // to determine closest way to go to target

        delta_angle = fmod(delta_angle + 180, 360) - 180; // wrap into -180 to 180

        // if obstacle is in left from robot
        if (delta_angle < 0)
        {
            // Turn robot to right, (90 degrees from obs)
            ret.vx = translation_vel * cos((obs.angle - 90) * DEG2RAD);
            ret.vy = translation_vel * sin((obs.angle - 90) * DEG2RAD);
        }
        // if obstacle is in right from robot
        else
        {
            // Turn robot to left, (90 degrees from obs)
            ret.vx = translation_vel * cos((obs.angle + 90) * DEG2RAD);
            ret.vy = translation_vel * sin((obs.angle + 90) * DEG2RAD);
        }
    }
    else // if no obs, then just use current vel
    {
        ret.vx = vx_input;
        ret.vy = vy_input;
    }

    ret.angle = obs.angle;
    ret.distance = obs.distance;
    ret.status = obs.status;

    return ret;
}

bool PotentialMotion(float target_x, float target_y, float target_th, float vel_position, float vel_th, int16_t flags, Robot *ret)
{
    AssignRobotTarget(target_x, target_y, target_th);
    AssignRawVelocity(vel_position, 0);

    static PID_t angles_pid;
    PIDInit(&angles_pid, 3.2, 0, 0);

    VectorAttractive vec_attr;
    vec_attr.init(attr_rad, vel_position);
    float r, theta;
    vec_attr.update(robot.pose_x, robot.pose_x, target_x, target_y, r, theta);
    output.x = r * cos(theta);
    output.y = r * sin(theta);

    //--------------------------------------------------------------------------------

    VectorRepulsive vec_rep;
    vec_rep.init(repl_rad, vel_position);
    for (uint8_t i = 0; i < dumped_obs_on_field.size(); i++)
    {
        if (dumped_obs_on_field[i] == 0 || dumped_obs_on_field[i] == 9999)
            continue;
        float pos_x_obs = robot.pose_x + dumped_obs_on_field[i] * cos(i * 2.5 * DEG2RAD);
        float pos_y_obs = robot.pose_x + dumped_obs_on_field[i] * sin(i * 2.5 * DEG2RAD);

        vec_rep.update(robot.pose_x, robot.pose_x, pos_x_obs, pos_y_obs, r, theta);
        output.x += r * cos(theta);
        output.y += r * sin(theta);
    }

    //--------------------------------------------------------------------------------

    if ((flags & normal_obstacle_ball) == normal_obstacle_ball)
    {
        vec_rep.update(robot.pose_x, robot.pose_x, ball.pose_x, ball.pose_y, r, theta);
        output.x += r * cos(theta);
        output.y += r * sin(theta);
    }

    //--------------------------------------------------------------------------------

    if (output.x < 20 && output.y < 20)
    {
        return MotionToPoint(target_x, target_y, target_th, vel_position, vel_th, flags);
    }

    //--------------------------------------------------------------------------------

    error.x = target_x - robot.pose_x;
    /* Error Y */
    error.y = target_y - robot.pose_x;
    /* Error Position */
    error.pose = sqrt(error.x * error.x + error.y * error.y);

    error.th = fmod(target_th - robot.pose_th + 180, 360) - 180;

    output.th = PIDCalculate(&angles_pid, error.th, vel_th);

    ManualMotionPosition(output.x, output.y, output.th, ret);

    return (fabs(error.pose) < 20 && fabs(error.th) < 10);
}