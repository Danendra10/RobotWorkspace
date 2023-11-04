#ifndef MAIN_HPP
#define MAIN_HPP

#define DESIRED_FREQUENCY 60

#pragma once

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <std_msgs/UInt8MultiArray.h>

#include <msg_collection/ThresholdVision.h>

#include <boost/thread/mutex.hpp>
#include <thread>
#include "opencv2/opencv.hpp"

#include "logger/logger.hpp"
#include "vision_helper/vision_helper.hpp"
#include "global/global.hpp"

logger::Logger logger_instance;

//---Vision Variables
cv::VideoCapture cap;

cv::Mat recieved_frame;
cv::Mat raw_frame;
cv::Mat bgr_frame;
cv::Mat thresholded_field;
cv::Mat raw_thresholded_field;
cv::Mat raw_ball;
cv::Mat thresholded_ball;
cv::Mat display_field;

//---ROS Variables
ros::Timer main_tim;

image_transport::Subscriber sub_raw_frame;

image_transport::Publisher pub_display_frame;
image_transport::Publisher pub_thresholded_field;
image_transport::Publisher pub_thresholded_ball;
image_transport::Publisher pub_thresholded_line;

ros::ServiceServer srv_vision_field_threshold_params;
ros::ServiceServer srv_vision_ball_threshold_params;

ros::Subscriber sub_vision_field_threshold_params;
ros::Subscriber sub_vision_ball_threshold_params;

//---Variables
std::mutex mtx_main_frame;
std::mutex mtx_field_frame;
std::mutex mtx_ball_frame;
std::mutex mtx_ball_and_field_frame;
std::mutex mtx_draw_field;
std::mutex mtx_bgr_field;

const uint16_t res_x = 360;
const uint16_t res_y = 640;
const uint8_t r_cam = 60;
const uint16_t center_cam_x = res_x * 0.5;
const uint16_t center_cam_y = res_y * 0.5;
uint16_t counter_ball_in;
uint16_t counter_ball_out;
uint8_t ball_status;
Ball_t ball_on_frame;
Ball_t ball_on_field;
Robot_t robot_on_field;

int field_threshold[6] = {53, 101, 61, 255, 116, 255};
int ball_threshold[6] = {1, 53, 73, 255, 123, 255};
int line_threshold[6] = {37, 82, 152, 255, 0, 155};

//---Callback
void CllbckMain(const ros::TimerEvent &event);
void CllbckSubRawFrame(const sensor_msgs::ImageConstPtr &msg);
void CllbckSubVisionFieldThresholdParams(const std_msgs::UInt8MultiArray::ConstPtr &msg);
void CllbckSubVisionBallThresholdParams(const std_msgs::UInt8MultiArray::ConstPtr &msg);
bool SrvVisionFieldThresholdParams(msg_collection::ThresholdVision::Request &req, msg_collection::ThresholdVision::Response &res);
bool SrvVisionBallThresholdParams(msg_collection::ThresholdVision::Request &req, msg_collection::ThresholdVision::Response &res);

//---Prototypes
int Init();
int Routine();
void TransmitFrame();

#endif