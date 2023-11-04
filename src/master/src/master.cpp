#include "master/master.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(8);

    //---Init
    if (Init(argc, argv) != 0)
    {
        ROS_ERROR("Failed to init master.");
        return -1;
    }

    //---Timer
    main_tim = nh.createTimer(ros::Duration(1.0 / DESIRED_FREQUENCY), CllbckMain);

    spinner.spin();
    main_tim.stop();
    return 0;
}

void CllbckMain(const ros::TimerEvent &event)
{
    try
    {
        if (ros::ok())
        {
            KeyboardHandler();
            roles[ATT]();
        }
        else
            throw std::runtime_error("ROS is not ok.");
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

//---Other Functions
int Init(int argc, char **argv)
{
    std::string robot_num = getenv("ROBOT_NUM");
    robot_number = atoi(robot_num.c_str());

    logger_instance.Log(logger::YELLOW, "Robot number: %d", robot_number);

    return 0;
}

int8_t Kbhit()
{
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized)
    {
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

/**
 * This function is used to handle keyboard input based on STDIN
 */
void KeyboardHandler()
{
    static uint8_t prev_key = 0;
    if (Kbhit() > 0)
    {
        char key = std::cin.get();

        switch (key)
        {
        case 'q':
            logger_instance.Log(logger::YELLOW, "Quitting...");
            break;
        case 'Q':
            logger_instance.Log(logger::BLUE, "Quitting...");
            break;
        }
        robot_base_action = (key != 'S' && key != ' '); // Set base_act to 0 if key == S or space
    }
}