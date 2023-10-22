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
    // logger_instance.Log(logger::GREEN, "Hello World!");
}

//---Other Functions
int Init(int argc, char **argv)
{
    //---Init Logger
    logger::Logger logger;

    return 0;
}