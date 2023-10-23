/**
 * @author IRIS-ITS
 * @brief this node will only work if main program is running correctly
 * @param robot_velocity with floating point numbers
 *
 * Datasheets:
 * - https://www.pk-rus.ru/fileadmin/download/simpleiq_command_referense_manual.pdf
 * - https://www.motorpowerco.com/media/filer_public/e6/6b/e66b3d73-815d-465d-af5a-f4379ae9d8c4/man-solgui.pdf
 * */

#include <atomic>
#include <ros/ros.h>
#include "comm/rs232.h"
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include <iris_msgs/stm2pc.h>
#include "geometry_msgs/Twist.h"
#include "iris_msgs/enc_motor.h"
#include "boost/thread/mutex.hpp"
#include "geometry_msgs/Vector3.h"
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#define BAUD_RATE 57600
#define DEG2RAD 0.017452925
#define RAD2DEG 57.295780
#define ENC2CM 0.0000202
#define ENC2CM_X 0.0000310 // Kacek 5
#define ENC2CM_Y 0.0000121 // Kacek 5
#define ENC2DEG 0.0000259707
#define CNTS2DEG (360.0f / 13186.0f)
#define DEG2CNTS (13186.0f / 360.0f)
#define RAD2CNTS (13186.0f * 0.5 / M_PI)
#define MOTOR2CENTER 22.4
#define MOTOR_RADIUS 6.3
#define AUX_CONST 1
#define DEAD_THRESH 1

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
float add_vels[2];

// const char control_cmd[64] = "PP[2]=4;PP[4]=0;IL[3]=7;IL[4]=7;UM=5;MO=1;";
const char control_cmd[64] = "IL[3]=3;UM=2;MO=1;";

ros::Subscriber sub_vel_motor;
ros::Subscriber sub_stm2pc;
ros::Subscriber sub_additional_vels;
ros::Publisher pub_vel_dribble;
ros::Publisher pub_enc_motor;

// STM datas
std::atomic<uint8_t> toggle3_pressed(0);

/**
 * Config:
 * Port motor from 16 to 21
 * PI const for each motors
 */
uint8_t left_motor;
uint8_t right_motor;
uint8_t rear_motor;
int k_PI_left[2];
int k_PI_right[2];
int k_PI_rear[2];

void loadConfig();
void CllbckVelMotor(const geometry_msgs::TwistConstPtr& msg);
void CllbckSubStm2Pc(const iris_msgs::stm2pcConstPtr& msg);
void DribbleAccelerate(short int* pv, short int sp, short int acceleration);
void GetMotorEnc(unsigned char* raw, uint8_t size, encoder_t* enc_);
void CllbckAddVels(const std_msgs::Float32MultiArrayConstPtr& msg);

void control_motor(const ros::TimerEvent&)
{
    static uint8_t local_toggle3_pressed;
    static uint8_t prev_motor_state1 = 0;
    static uint8_t prev_motor_state2 = 0;
    static uint8_t prev_motor_state3 = 0;

    static encoder_t rear_motor_enc;
    static encoder_t left_motor_enc;
    static encoder_t right_motor_enc;

    rear_motor_enc.angle = 0;
    left_motor_enc.angle = 240;
    right_motor_enc.angle = 120;

    unsigned char read_buf1[512];
    unsigned char read_buf2[512];
    unsigned char read_buf3[512];

    /* Get all data from motors */
    uint8_t nrecv2 = RS232_PollComport(left_motor, read_buf2, sizeof(read_buf2));
    GetMotorEnc(read_buf2, nrecv2, &left_motor_enc);
    uint8_t nrecv3 = RS232_PollComport(right_motor, read_buf3, sizeof(read_buf3));
    GetMotorEnc(read_buf3, nrecv3, &right_motor_enc);
    uint8_t nrecv1 = RS232_PollComport(rear_motor, read_buf1, sizeof(read_buf1));
    GetMotorEnc(read_buf1, nrecv1, &rear_motor_enc);

    RS232_flushRX(left_motor);
    RS232_flushRX(right_motor);
    RS232_flushRX(rear_motor);

    /* It will auto restart motor if motor was dead (Rising Edge Detection) */
    if (prev_motor_state1 == 0 && nrecv1 > 0)
        RS232_cputs(rear_motor, control_cmd);
    if (prev_motor_state2 == 0 && nrecv2 > 0)
        RS232_cputs(left_motor, control_cmd);
    if (prev_motor_state3 == 0 && nrecv3 > 0)
        RS232_cputs(right_motor, control_cmd);
    // printf("%s\n%s\n%s\n", read_buf1, read_buf2, read_buf3);


    // printf("satu %s\n", read_buf1);
    // printf("dua %s\n", read_buf2);
    // printf("tiga %s\n", read_buf3);

    // printf("=========================================\n");

    float vx = (rear_motor_enc.x_robot + left_motor_enc.x_robot + right_motor_enc.x_robot) * ENC2CM;
    float vy = (rear_motor_enc.y_robot + left_motor_enc.y_robot + right_motor_enc.y_robot) * ENC2CM;
    float vth = (rear_motor_enc.th + left_motor_enc.th + right_motor_enc.th) * ENC2DEG;

    static float vel_left_motor;
    static float vel_right_motor;
    static float vel_rear_motor;

    left_motor_enc.angle = 240;
    right_motor_enc.angle = 120;
    rear_motor_enc.angle = 0;

    float used_vth = 0;
    float used_vy = 0;


    float add_vth = 0;
    float add_vforward = 0;

    float error_throttle = add_vels[0] - add_vels[1];
    // printf("ERROR THROTTLE %.2f\n", error_throttle);
    add_vth = error_throttle / 5;
    add_vforward = fabs(error_throttle) / 6;

    used_vth = vel_th * DEG2RAD * MOTOR2CENTER * AUX_CONST + add_vth;

    // printf("ERROR THROTTLE %.2f\n", used_vth);

    used_vy = vel_y + add_vforward;


    /* Addition vel to handle the ball, if ball is left from robot, so robot will turn left. Based on throttle*/
    // if (fabs(add_vels[0]) > fabs(add_vels[1]))
    //     add_vth = add_vels[0] * 1 / 50;
    // else
    //     add_vth = add_vels[1] * 1 /60;

    // Convert angular vel_th to linear for each motors. L = a . r
    // used_vth = vel_th * DEG2RAD * MOTOR2CENTER * AUX_CONST + add_vth;

    vel_left_motor = used_vth + vel_x * cosf(left_motor_enc.angle * DEG2RAD) + used_vy * sinf(left_motor_enc.angle * DEG2RAD);
    vel_right_motor = used_vth + vel_x * cosf(right_motor_enc.angle * DEG2RAD) + used_vy * sinf(right_motor_enc.angle * DEG2RAD);
    vel_rear_motor = used_vth + vel_x;

    /* Convert linear velocity to Angular velocity (cnt/s) for each motor */
    vel_left_motor = vel_left_motor / MOTOR_RADIUS * RAD2CNTS * 1;
    vel_right_motor = vel_right_motor / MOTOR_RADIUS * RAD2CNTS * 1;
    vel_rear_motor = vel_rear_motor / MOTOR_RADIUS * RAD2CNTS * 1; // 6959

    char cmd_mtr_left[128];
    char cmd_mtr_right[128];
    char cmd_mtr_rear[128];

    local_toggle3_pressed = toggle3_pressed.load();
    // local_toggle3_pressed = 1;

    iris_msgs::enc_motor msg_px_motor;
    msg_px_motor.left_px = left_motor_enc.px;
    msg_px_motor.right_px = right_motor_enc.px;
    msg_px_motor.rear_px = rear_motor_enc.px;
    msg_px_motor.left_vx = left_motor_enc.base_enc;
    msg_px_motor.right_vx = right_motor_enc.base_enc;
    msg_px_motor.rear_vx = rear_motor_enc.base_enc;
    msg_px_motor.left_nrecv = (nrecv2 * local_toggle3_pressed);
    msg_px_motor.rear_nrecv = (nrecv1 * local_toggle3_pressed);
    msg_px_motor.right_nrecv = (nrecv3 * local_toggle3_pressed);
    msg_px_motor.left_act_vx = vel_left_motor;
    msg_px_motor.right_act_vx = vel_right_motor;
    msg_px_motor.rear_act_vx = vel_rear_motor;

    // printf("ENCODER %d %d %d \n ", msg_px_motor.left_px, msg_px_motor.right_px, msg_px_motor.rear_px);
    pub_enc_motor.publish(msg_px_motor);

    /**
     * It will control motor,
     * if master node was dead, it will control motor to 0 velocity
     */
    if (ros::Time::now().toSec() - master_epoch <= DEAD_THRESH)
    {
        sprintf(cmd_mtr_rear, "MO=%d;PX;VX;BG;KP[2]=%d;KI[2]=%d;JV=%d;", local_toggle3_pressed, k_PI_rear[0], k_PI_rear[1], (int)(vel_rear_motor * local_toggle3_pressed));
        sprintf(cmd_mtr_left, "MO=%d;PX;VX;BG;KP[2]=%d;KI[2]=%d;JV=%d;", local_toggle3_pressed, k_PI_left[0], k_PI_left[1], (int)(vel_left_motor * local_toggle3_pressed));
        sprintf(cmd_mtr_right, "MO=%d;PX;VX;BG;KP[2]=%d;KI[2]=%d;JV=%d;", local_toggle3_pressed, k_PI_right[0], k_PI_right[1], (int)(vel_right_motor * local_toggle3_pressed));
    }
    else
    {
        sprintf(cmd_mtr_rear, "BG;KP[2]=%d;PX;VX;KI[2]=%d;MO=%d;JV=%d;", k_PI_rear[0], k_PI_rear[1], local_toggle3_pressed, (int)(0 * local_toggle3_pressed));
        sprintf(cmd_mtr_left, "BG;KP[2]=%d;PX;VX;KI[2]=%d;MO=%d;JV=%d;", k_PI_left[0], k_PI_left[1], local_toggle3_pressed, (int)(0 * local_toggle3_pressed));
        sprintf(cmd_mtr_right, "BG;KP[2]=%d;PX;VX;KI[2]=%d;MO=%d;JV=%d;", k_PI_right[0], k_PI_right[1], local_toggle3_pressed, (int)(0 * local_toggle3_pressed));
    }

    /* Send to MOTOR using serial communication */
    RS232_cputs(rear_motor, cmd_mtr_rear);
    RS232_cputs(left_motor, cmd_mtr_left);
    RS232_cputs(right_motor, cmd_mtr_right);

    RS232_flushTX(left_motor);
    RS232_flushTX(right_motor);
    RS232_flushTX(rear_motor);

    prev_motor_state1 = nrecv1;
    prev_motor_state2 = nrecv2;
    prev_motor_state3 = nrecv3;
}

void loadConfig()
{
    char* robot_num = getenv("ROBOT");
    char config_file[100];
    std::string current_dir = ros::package::getPath("comm");
    sprintf(config_file, "%s/../../config/IRIS%s/static_conf.yaml", current_dir.c_str(), robot_num);

    YAML::Node config = YAML::LoadFile(config_file);
    left_motor = config["Motor"]["left_motor_port"].as<int>();
    right_motor = config["Motor"]["right_motor_port"].as<int>();
    rear_motor = config["Motor"]["rear_motor_port"].as<int>();
    k_PI_left[0] = config["Motor"]["kp_left_motor"].as<int>();
    k_PI_left[1] = config["Motor"]["ki_left_motor"].as<int>();
    k_PI_right[0] = config["Motor"]["kp_right_motor"].as<int>();
    k_PI_right[1] = config["Motor"]["ki_right_motor"].as<int>();
    k_PI_rear[0] = config["Motor"]["kp_rear_motor"].as<int>();
    k_PI_rear[1] = config["Motor"]["ki_rear_motor"].as<int>();

    printf("PORT MOTOR: %d %d %d\n", left_motor, right_motor, rear_motor);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "comm_motor");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner spinner(4);

    char mode[4] = { '8', 'N', '1', 0 };

    loadConfig();

    if (RS232_OpenComport(left_motor, BAUD_RATE, mode))
    {
        printf("Can not open comport 1 (left) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        ros::shutdown();
    }
    if (RS232_OpenComport(right_motor, BAUD_RATE, mode))
    {
        printf("Can not open comport 2 (right) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        ros::shutdown();
    }
    if (RS232_OpenComport(rear_motor, BAUD_RATE, mode))
    {
        printf("Can not open comport 3 (rear) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        ros::shutdown();
    }

    RS232_enableDTR(left_motor);
    RS232_enableDTR(right_motor);
    RS232_enableDTR(rear_motor);

    tim_control_motor = NH.createTimer(ros::Duration(0.01), control_motor);

    sub_stm2pc = NH.subscribe("stm2pc", 1, CllbckSubStm2Pc);
    sub_vel_motor = NH.subscribe("cmd_vel_motor", 1, CllbckVelMotor);
    sub_additional_vels = NH.subscribe("additional_vel", 1, CllbckAddVels);
    pub_vel_dribble = NH.advertise<std_msgs::Int16MultiArray>("dribble_vel", 1);
    pub_enc_motor = NH.advertise<iris_msgs::enc_motor>("enc_motor", 2);

    spinner.spin();

    tim_control_motor.stop();

    return 0;
}

/* It get from Ballhandling */
void CllbckAddVels(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    add_vels[0] = msg->data[0];
    add_vels[1] = msg->data[1];
}

void CllbckVelMotor(const geometry_msgs::TwistConstPtr& msg)
{
    master_epoch = ros::Time::now().toSec();
    vel_x = msg->linear.x;
    vel_y = msg->linear.y;
    vel_th = msg->angular.z;

}
void CllbckSubStm2Pc(const iris_msgs::stm2pcConstPtr& msg)
{
    toggle3_pressed = !((msg->buttons & 0b10000000) >> 0x07);
}

// 0 1 2 3 4 5 6 
// 4 4 4 4 4 4 4
// 0 1 2 3 0 1 2

/**
 * @brief Combine three encoders with IMU from MPU6050
 * @param raw data from driver
 * @param size data size from driver
 * @param enc_ encoder struct
 */
void GetMotorEnc(unsigned char* raw, uint8_t size, encoder_t* enc_)
{
    unsigned char enc_buff[32] = { '0' }; // 0 default value
    unsigned char px_buff[32] = { '0' };  // 0 default value
    uint8_t cnt_found = 0;

    /**
     * Extract encoder from driver
     * Encoder velocity data is "VX:<data>"
     * Encoder position data is "PX:<data>"
     *
     * */
    for (uint8_t i = 0; i < size; i++)
    {
        if (raw[i] == 0x56) // V
        {
            if (raw[i + 1] == 0x58) // X
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

                    if (raw[j] == 0x3B) // ;
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
                    enc_->base_enc = atoi((const char*)enc_buff);
                }
                cnt_found++;
            }
        }
        if (raw[i] == 0x50) // P
        {
            if (raw[i + 1] == 0x58) // X
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
                    if (raw[j] == 0x3B) // ;
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
                    enc_->px = atoi((const char*)px_buff);
                }
                cnt_found++;
            }
        }

        if (cnt_found == 0x02)
            break;
    }

    return;
}