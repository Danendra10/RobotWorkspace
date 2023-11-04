#ifndef MASTER_HPP_
#define MASTER_HPP_

#pragma once

#include <ros/ros.h>

//---Addons (helpers)
#include <math/simple_math.hpp>
#include <logger/logger.hpp>
#include <motion/motion.hpp>
#include <multirole/multirole.h>
#include <global/global.hpp>

//---Addons (Roles)
#include <role_attacker/role_attacker.hpp>

//---Addons
#include <termios.h>
#include <sys/ioctl.h>

#define DESIRED_FREQUENCY 60

//---Timer
ros::Timer main_tim;

//---Variables
logger::Logger logger_instance;

//---Declared on global.hpp
extern uint8_t robot_base_action;

//---Roles
MultiRole roles[] = {
    AttRun,
};

//---Robot Variables
uint8_t robot_number;
extern Robot robot;

//---Callback
void CllbckMain(const ros::TimerEvent &event);

//---Prototypes
int Init(int argc, char **argv);
int Routine();

//---Utils
int8_t Kbhit();
void KeyboardHandler();

#endif // MASTER_HPP_