#ifndef ROLE_ATTACKER_HPP_
#define ROLE_ATTACKER_HPP_

#pragma once

#include <stdio.h>
#include <string.h>
#include <stdint.h>

//--->IRIS's packages
#include <multirole/multirole.h>
#include <motion/motion.hpp>
#include <helper/helper.hpp>
#include <math/simple_math.hpp>

//---Game States
extern uint8_t game_status;
extern uint8_t has_prep;

//-->Dribble
extern uint8_t dribble_up;
extern uint8_t long_pull_dribble;

void AttRun();
#endif