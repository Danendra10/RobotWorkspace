#ifndef CAPTURE_HPP_
#define CAPTURE_HPP_

#define DESIRED_FREQUENCY 60

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>

#include <boost/thread/mutex.hpp>
#include <thread>

#include "logger/logger.hpp"
#include "opencv2/opencv.hpp"
#include <helper/helper.hpp>
#include <string>

logger::Logger logger_instance;

//---Vision Variables
cv::VideoCapture cap;

cv::Mat main_frame;

//---ROS Variables
ros::Timer main_tim;

image_transport::Publisher pub_frame;

//---Variables
std::mutex mtx_main_frame;
const uint16_t res_x = 360;
const uint16_t res_y = 640;
std::string camera_path;

//---Callback
void CllbckMain(const ros::TimerEvent &event);

//---Prototypes
int Init();
int Routine();
void InitializeCaptureProperties();
void ReopenCapture();
#endif