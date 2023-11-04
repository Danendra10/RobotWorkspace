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

    spinner.spin();

    return 0;
}

void CllbckMain(const ros::TimerEvent &event)
{
    bool frameRead = cap.read(main_frame); // Use read instead of >> for a clearer check
#ifdef DEVELOPMENT
    if (frameRead && !main_frame.empty())
    {
        mtx_main_frame.lock();
        cap >> main_frame;
        mtx_main_frame.unlock();
        if (!main_frame.empty())
        {
            cv::flip(main_frame, main_frame, 0);
            cv::resize(main_frame, main_frame, cv::Size(res_y, res_x));
            cv::rotate(main_frame, main_frame, cv::ROTATE_90_COUNTERCLOCKWISE);
            cv::Mat send_buf;
            send_buf = main_frame.clone();
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", send_buf).toImageMsg();
            pub_frame.publish(msg);
        }
    }
    else
    {
        ROS_INFO("Reached the end of the video or failed to read frame. Attempting to reopen.");

        // Attempt to reopen the video capture
        ReopenCapture();
    }
#else
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
            cv::Mat send_buf;
            send_buf = main_frame.clone();
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", send_buf).toImageMsg();
            pub_frame.publish(msg);
        }
    }
    else
    {
        ROS_ERROR("Failed to open camera.");
    }
#endif
}

int Init()
{
    try
    {
#ifdef DEVELOPMENT
        camera_path = ros::package::getPath("vision") + "/development_video/field.mp4";
#else
        camera_path = GetEnv("CAMERA_PATH");
#endif
        ROS_ERROR("Camera path: %s", camera_path.c_str());
        cap.open(camera_path);

        if (!cap.isOpened())
        {
            throw std::runtime_error("Failed to open camera.");
        }

        InitializeCaptureProperties();

        return 0;
    }
    catch (const std::exception &e)
    {
        logger_instance.Log(logger::RED, e.what());
        return -1;
    }
}

void InitializeCaptureProperties()
{
    // Set capture properties, as done in Init()
    cap.set(cv::CAP_V4L2, 1);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FPS, 60);
}

void ReopenCapture()
{
    // Optionally release the current capture object
    cap.release();

    // Then attempt to reopen
    cap.open(camera_path);
    if (!cap.isOpened())
    {
        ROS_ERROR("Failed to reopen camera.");
        // Handle the error, perhaps attempt to reopen after a delay, or exit
    }
    else
    {
        // Re-initialize capture properties if needed
        InitializeCaptureProperties();
        ROS_INFO("Successfully reopened the camera.");
    }
}