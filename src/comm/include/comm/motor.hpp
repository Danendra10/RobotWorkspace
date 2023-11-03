#ifndef MOTOR_HPP_
#define MOTOR_HPP_

/**
 * @author IRIS-ITS
 * @brief this node will only work if main program is running correctly
 * @param robot_velocity with floating point numbers
 * Datasheets:
 * - https://www.pk-rus.ru/fileadmin/download/simpleiq_command_referense_manual.pdf
 * - https://www.motorpowerco.com/media/filer_public/e6/6b/e66b3d73-815d-465d-af5a-f4379ae9d8c4/man-solgui.pdf
 * */

#include <ros/ros.h>
#include "comm/rs232.h"
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include "boost/thread/mutex.hpp"
#include <msg_collection/stmToPc.h>
#include "geometry_msgs/Twist.h"
#include "msg_collection/encMotor.h"
#include <std_msgs/Int16MultiArray.h>
#include "geometry_msgs/Vector3.h"

#define BAUD_RATE 57600
#define DEG2RAD 0.017452925
#define RAD2DEG 57.295780
#define _SIGN(x, _sign) (_sign * x + !_sign * -x)
#define DCNTS100HZ_MAX 360
#define UINT16_RANGE (0xFFFF + 1)
#define DABS(a) ((a) >= 0 ? (a) : -(a))
#define ENC2CM 0.0000202
#define ENC2CM_X 0.0000310 // Kacek 5
#define ENC2CM_Y 0.0000121 // Kacek 5
#define ENC2DEG 0.0000259707
#define CNTS2DEG (360.0f / 13186.0f)
#define DEG2CNTS (13186.0f / 360.0f)
#define AUX_CONST 1.25

#define DEFAULT_MOTOR_GAIN 20
#define LEFT_MOTOR_GAIN DEFAULT_MOTOR_GAIN
#define RIGHT_MOTOR_GAIN DEFAULT_MOTOR_GAIN
#define REAR_MOTOR_GAIN DEFAULT_MOTOR_GAIN
typedef struct encoder_tag
{
    int16_t base_enc;
    int16_t angle;

    /* By velocity */
    float x;
    float y;
    float th;

    /* By position */
    int64_t px;
    int64_t prev_px;
    float v_fpx;

    uint8_t px_set;
    uint8_t px_set_cnt;

    /* Based on robot */
    float x_robot;
    float y_robot;
} encoder_t;

typedef struct pose_tag
{
    float x;
    float y;
    float th;
} pose_t;

ros::Timer tim_control_motor;

float vel_x = 0;
float vel_y = 0;
float vel_th = 0;
double master_epoch;

const char control_cmd[64] = "IL[3]=7;IL[4]=7;UM=5;MO=1;";

ros::Subscriber sub_vel_motor;
ros::Subscriber sub_stm2pc;
ros::Publisher pub_vel_dribble;
ros::Publisher pub_enc_motor;

// STM datas
uint8_t toggle1_pressed;
uint8_t toggle2_pressed;
// uint8_t toggle3_pressed;

uint8_t toggle3_pressed = 0;
float gyro_buffer = 999999;

/**
 * Config:
 * Port motor from 16 to 21
 * PI const for each motors
 */
uint8_t left_motor;
uint8_t right_motor;
uint8_t rear_motor;
float k_PI_left[2];
float k_PI_right[2];
float k_PI_rear[2];

void loadConfig();
void CllbckVelMotor(const geometry_msgs::TwistConstPtr &msg);
void CllbckSubStm2Pc(const msg_collection::stmToPcConstPtr &msg);
void DribbleAccelerate(short int *pv, short int sp, short int acceleration);
void GetMotorEnc(unsigned char *raw, uint8_t size, encoder_t *enc_);

void control_motor(const ros::TimerEvent &)
{

    static uint8_t prev_motor_state1 = 0;
    static uint8_t prev_motor_state2 = 0;
    static uint8_t prev_motor_state3 = 0;

    static encoder_t rear_motor_enc;
    static encoder_t left_motor_enc;
    static encoder_t right_motor_enc;

    rear_motor_enc.angle = 0;
    left_motor_enc.angle = 239;
    right_motor_enc.angle = 120;

    unsigned char read_buf1[512];
    unsigned char read_buf2[512];
    unsigned char read_buf3[512];

    uint8_t nrecv1 = RS232_PollComport(rear_motor, read_buf1, sizeof(read_buf1));
    GetMotorEnc(read_buf1, nrecv1, &rear_motor_enc);
    uint8_t nrecv2 = RS232_PollComport(left_motor, read_buf2, sizeof(read_buf2));
    GetMotorEnc(read_buf2, nrecv2, &left_motor_enc);
    uint8_t nrecv3 = RS232_PollComport(right_motor, read_buf3, sizeof(read_buf3));
    GetMotorEnc(read_buf3, nrecv3, &right_motor_enc);

    if (prev_motor_state1 == 0x00 && nrecv1 > 0b00)
        RS232_cputs(rear_motor, control_cmd);
    if (prev_motor_state2 == 0x00 && nrecv2 > 0b00)
        RS232_cputs(left_motor, control_cmd);
    if (prev_motor_state3 == 0x00 && nrecv3 > 0b00)
        RS232_cputs(right_motor, control_cmd);

    float vx = (rear_motor_enc.x_robot + left_motor_enc.x_robot + right_motor_enc.x_robot) * ENC2CM;
    float vy = (rear_motor_enc.y_robot + left_motor_enc.y_robot + right_motor_enc.y_robot) * ENC2CM;
    float vth = (rear_motor_enc.th + left_motor_enc.th + right_motor_enc.th) * ENC2DEG;

    msg_collection::encMotor msg_px_motor;
    msg_px_motor.left_px = left_motor_enc.px;
    msg_px_motor.right_px = right_motor_enc.px;
    msg_px_motor.rear_px = rear_motor_enc.px;
    msg_px_motor.left_vx = left_motor_enc.base_enc;
    msg_px_motor.right_vx = right_motor_enc.base_enc;
    msg_px_motor.rear_vx = rear_motor_enc.base_enc;

    pub_enc_motor.publish(msg_px_motor);

    static float vel_left_motor;
    static float vel_right_motor;
    static float vel_rear_motor;

    left_motor_enc.angle = 240;
    right_motor_enc.angle = 120;
    rear_motor_enc.angle = 0;

    // printf("vx_sp___: %.2f %.2f\n", vel_x, vel_y);

    // Convert angular vel_th to linear for each motors. L = a . r
    // vel_th = vel_th * DEG2RAD * 24 * AUX_CONST; // 24 iki jarak pusat ke roda

    vel_left_motor = vel_th + vel_x * cosf(left_motor_enc.angle * DEG2RAD) + vel_y * sinf(left_motor_enc.angle * DEG2RAD);
    vel_right_motor = vel_th + vel_x * cosf(right_motor_enc.angle * DEG2RAD) + vel_y * sinf(right_motor_enc.angle * DEG2RAD);
    vel_rear_motor = vel_th + vel_x;

    vel_left_motor = vel_left_motor / 6.15 * RAD2DEG * DEG2CNTS;
    vel_right_motor = vel_right_motor / 6.15 * RAD2DEG * DEG2CNTS;
    vel_rear_motor = vel_rear_motor / 6.15 * RAD2DEG * DEG2CNTS; // 6959

    // asd = qwe / r * mpi / 180 * 540 / 360

    char cmd_mtr_left[64];
    char cmd_mtr_right[64];
    char cmd_mtr_rear[64];

    if (ros::Time::now().toSec() - master_epoch <= 1.5)
    {
        sprintf(cmd_mtr_rear, "JV=%d;BG;KP[2]=%f;PX;VX;KI[2]=%f;MO=%d;", (int)(vel_rear_motor * toggle3_pressed), k_PI_rear[0], k_PI_rear[1], toggle3_pressed);
        sprintf(cmd_mtr_left, "JV=%d;BG;KP[2]=%f;PX;VX;KI[2]=%f;MO=%d;", (int)(vel_left_motor * toggle3_pressed), k_PI_left[0], k_PI_left[1], toggle3_pressed);
        sprintf(cmd_mtr_right, "JV=%d;BG;KP[2]=%f;PX;VX;KI[2]=%f;MO=%d;", (int)(vel_right_motor * toggle3_pressed), k_PI_right[0], k_PI_right[1], toggle3_pressed);
    }
    else
    {
        sprintf(cmd_mtr_rear, "JV=%d;BG;KP[2]=%f;PX;VX;KI[2]=%f;MO=%d;", (int)(0 * toggle3_pressed), k_PI_rear[0], k_PI_rear[1], toggle3_pressed);
        sprintf(cmd_mtr_left, "JV=%d;BG;KP[2]=%f;PX;VX;KI[2]=%f;MO=%d;", (int)(0 * toggle3_pressed), k_PI_left[0], k_PI_left[1], toggle3_pressed);
        sprintf(cmd_mtr_right, "JV=%d;BG;KP[2]=%f;PX;VX;KI[2]=%f;MO=%d;", (int)(0 * toggle3_pressed), k_PI_right[0], k_PI_right[1], toggle3_pressed);
    }

    // toggle3_pressed = 0;
    // sprintf(cmd_mtr_rear, "PX;VX;JV=%d;BG;KP[2]=%f;KI[2]=%f;MO=%d;", (int)(vel_rear_motor * toggle3_pressed), k_PI_rear[0], k_PI_rear[1], toggle3_pressed);
    // sprintf(cmd_mtr_left, "PX;VX;JV=%d;BG;KP[2]=%f;KI[2]=%f;MO=%d;", (int)(vel_left_motor * toggle3_pressed), k_PI_left[0], k_PI_left[1], toggle3_pressed);
    // sprintf(cmd_mtr_right, "PX;VX;JV=%d;BG;KP[2]=%f;KI[2]=%f;MO=%d;", (int)(vel_right_motor * toggle3_pressed), k_PI_right[0], k_PI_right[1], toggle3_pressed);

    RS232_cputs(rear_motor, cmd_mtr_rear);
    RS232_cputs(left_motor, cmd_mtr_left);
    RS232_cputs(right_motor, cmd_mtr_right);

    prev_motor_state1 = nrecv1;
    prev_motor_state2 = nrecv2;
    prev_motor_state3 = nrecv3;
}

void loadConfig()
{
    char *robot_num = getenv("ROBOT");
    char config_file[100];
    std::string current_dir = ros::package::getPath("comm");
    sprintf(config_file, "%s/../../config/IRIS%s/motor.yaml", current_dir.c_str(), robot_num);

    YAML::Node config = YAML::LoadFile(config_file);
    left_motor = config["left_motor_port"].as<int>();
    right_motor = config["right_motor_port"].as<int>();
    rear_motor = config["rear_motor_port"].as<int>();
    k_PI_left[0] = config["kp_left_motor"].as<float>();
    k_PI_left[1] = config["ki_left_motor"].as<float>();
    k_PI_right[0] = config["kp_right_motor"].as<float>();
    k_PI_right[1] = config["ki_right_motor"].as<float>();
    k_PI_rear[0] = config["kp_rear_motor"].as<float>();
    k_PI_rear[1] = config["ki_rear_motor"].as<float>();

    printf("PORT MOTOR: %d %d %d\n", left_motor, right_motor, rear_motor);
}

void CllbckVelMotor(const geometry_msgs::TwistConstPtr &msg)
{
    master_epoch = ros::Time::now().toSec();
    vel_x = msg->linear.x;
    vel_y = msg->linear.y;
    vel_th = msg->angular.z;
}
void CllbckSubStm2Pc(const msg_collection::stmToPcConstPtr &msg)
{
    toggle1_pressed = !((msg->buttons & 0b00100000) >> 0x05);
    toggle2_pressed = !((msg->buttons & 0b01000000) >> 0x06);
    toggle3_pressed = !((msg->buttons & 0b10000000) >> 0x07);

    // toggle3_pressed = 0;

    gyro_buffer = msg->gyro_buffer;
}

/**
 * @brief Combine three encoders with IMU from MPU6050
 * @param raw data from driver
 * @param size data size from driver
 * @param enc_ encoder struct
 */
void GetMotorEnc(unsigned char *raw, uint8_t size, encoder_t *enc_)
{
    unsigned char enc_buff[32] = {'0'}; // 0 default value
    unsigned char px_buff[32] = {'0'};  // 0 default value
    uint8_t cnt_found = 0;

    // printf("raw: %s\n", raw);

    /* Extract encoder from driver */
    for (uint8_t i = 0; i < size; i++)
    {
        if (raw[i] == 0x56)
        {

            if (raw[i + 1] == 0x58)
            {
                uint8_t buff_ctr = 0;
                uint8_t expected_count = -1;

                for (uint8_t j = i + 3; j < size; j++)
                {
                    /* Filter string from wild char */
                    if ((raw[j] >= '0' && raw[j] <= '9') || raw[j] == '-')
                    {
                        enc_buff[buff_ctr] = raw[j];
                        buff_ctr++;
                    }
                    expected_count++;

                    if (raw[j] == 0x3B)
                    {
                        break;
                    }
                }
                /* Filter string from wild char */
                if (buff_ctr != expected_count)
                {
                    // ROS_ERROR("DATA MOTOR BODOH (VX) (%d %d) %s", expected_count, buff_ctr, enc_buff);
                    enc_->base_enc = enc_->base_enc;
                }
                else
                {
                    // ROS_WARN("GET (%d %d) %s", expected_count, buff_ctr, enc_buff);
                    enc_buff[buff_ctr] = '\0'; // Null terminated string
                    enc_->base_enc = atoi((const char *)enc_buff);
                }
                cnt_found++;
            }
        }
        if (raw[i] == 0x50)
        {
            // ROS_ERROR("jancok asu %d -> %d", i, size);
            if (size - i < 20) // Tolong siapapun fix iki rekk... :(
            {
                enc_->px = enc_->px;
                cnt_found++;

                break;
            }
            if (raw[i + 1] == 0x58)
            {
                uint8_t buff_ctr = 0;
                uint8_t expected_count = -1;
                for (uint8_t j = i + 3; j < size; j++)
                {
                    /* Filter string from wild char */
                    if ((raw[j] >= '0' && raw[j] <= '9') || raw[j] == '-')
                    {
                        px_buff[buff_ctr] = raw[j];
                        buff_ctr++;
                    }
                    expected_count++;
                    if (raw[j] == 0x3B)
                    {
                        break;
                    }
                }
                /* Filter string from wild char */
                if (buff_ctr != expected_count)
                {
                    // ROS_ERROR("DATA MOTOR BODOH (PX) (%d %d) %s", expected_count, buff_ctr, px_buff);
                    enc_->px = enc_->px;
                }
                else
                {
                    px_buff[buff_ctr] = '\0'; // Null terminated string
                    enc_->px = atoi((const char *)px_buff);
                }
                cnt_found++;
            }
        }

        if (cnt_found == 0x02)
            break;
    }

    return;
}

#endif