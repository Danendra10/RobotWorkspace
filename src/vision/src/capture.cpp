#include "capture/capture.hpp"

// #define DEBUG
int main(int argc, char **argv)
{
    ros::init(argc, argv, "capture");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::MultiThreadedSpinner spinner(4); // Reduced to 4 threads

    if (Init() != 0)
    {
        ROS_ERROR("Failed to init capture.");
        return -1;
    }

    pub_frame = it.advertise("/vision/raw/frame", 1);
    main_tim = nh.createTimer(ros::Duration(1.0 / DESIRED_FREQUENCY), CllbckMain);

#ifdef DEBUG
    // Start spinner in another thread
    std::thread spinner_thread([&spinner]()
                               { spinner.spin(); });

    // GUI-related tasks on the main thread
    while (ros::ok())
    {
        if (!main_frame.empty())
        {
            //---Giving the IRIS's format of image processing
            cv::flip(main_frame, main_frame, 0);
            cv::resize(main_frame, main_frame, cv::Size(res_y, res_x));
            cv::rotate(main_frame, main_frame, cv::ROTATE_90_COUNTERCLOCKWISE);
            cv::imshow("Frame", main_frame);
            cv::waitKey(1);
        }
    }
    spinner_thread.join();
#else
    spinner.spin();
#endif

    return 0;
}

void CllbckMain(const ros::TimerEvent &event)
{
    if (cap.isOpened())
    {
        mtx_main_frame.lock();
        cap >> main_frame;
        mtx_main_frame.unlock();
        if (!main_frame.empty())
        {
            cv::flip(main_frame, main_frame, 0);
            cv::resize(main_frame, main_frame, cv::Size(res_y, res_x));
            cv::rotate(main_frame, main_frame, cv::ROTATE_90_COUNTERCLOCKWISE);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", main_frame).toImageMsg();
            pub_frame.publish(msg);
        }
    }
    else
    {
        ROS_ERROR("Failed to open camera.");
    }
}

int Init()
{
    try
    {
        cap.open("/Users/danendracleveroananda/Documents/Kuliah/TA/RobotWorkspace/IRIS_Beroda.mp4");

        if (!cap.isOpened())
        {
            throw std::runtime_error("Failed to open camera.");
        }
        cap.set(cv::CAP_V4L2, 1);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(cv::CAP_PROP_FPS, 60);

        return 0;
    }
    catch (const std::exception &e)
    {
        logger_instance.Log(logger::RED, e.what());
        return -1;
    }
}