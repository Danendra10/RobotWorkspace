#include <role_attacker/role_attacker.hpp>

void AttRun()
{
    printf("Attacker\n");
    // static uint8_t prev_;
    // static uint8_t prev_preparation = 0;
    // static uint64_t last_time_update = 0;

    // // has_prep 1 untuk langsung
    // if (game_status != prev_preparation || GetTimeNowMilliSec() - last_time_update > 2000)
    // {
    //     if (game_status > 66 && game_status < 85 && game_status != 78 && game_status != 80) // Prep homes
    //     {
    //         has_prep = 0;
    //     }
    //     else if (game_status > 98 && game_status < 117) // Prep away
    //     {
    //         has_prep = 2;
    //     }
    //     else if (game_status == 78) // dropball
    //     {
    //         has_prep = 1;
    //     }
    //     else if (game_status == status_preparation_penaltykick_home)
    //     {
    //         has_prep = 3;
    //     }

    //     prev_preparation = game_status;
    //     last_time_update = GetTimeNowMilliSec();

    //     dribble_up = 0;
    //     long_pull_dribble = 0;
    // }

    // switch (game_status * robot_base_action)
    // {
    // case status_iddle_2:
    //     ManualMotion(0, 0, 0, 3);
    //     break;

    // case keyboard_forward:
    //     ManualMotion(0, 100, 0, 3);
    //     break;

    // case keyboard_backward:
    //     ManualMotion(0, -100, 0, 3);
    //     break;

    // case keyboard_left:
    //     ManualMotion(-100, 0, 0, 3);
    //     break;

    // case keyboard_right:
    //     ManualMotion(100, 0, 0, 3);
    //     break;

    // case keyboard_left_rotation:
    //     // ManualMotion(75 * M_PI / 180 * 40, 0, 75, 3);
    //     ManualMotion(0, 0, 75, 3);
    //     break;

    // case keyboard_right_rotation:
    //     // ManualMotion(-75 * M_PI / 180 * 40, 0, -75, 3);
    //     ManualMotion(0, 0, -75, 3);
    //     break;
    // }

    return;
}