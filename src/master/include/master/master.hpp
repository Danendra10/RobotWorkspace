#ifndef MASTER_HPP_
#define MASTER_HPP_

#include "ros/ros.h"

//---Addons (helpers)
#include "logger/logger.hpp"

#define DESIRED_FREQUENCY 60

//---Timer
ros::Timer main_tim;

//---Variables
logger::Logger logger_instance;

//---Callback
void CllbckMain(const ros::TimerEvent &event);

//---Prototypes
int Init(int argc, char **argv);
int Routine();

#endif // MASTER_HPP_