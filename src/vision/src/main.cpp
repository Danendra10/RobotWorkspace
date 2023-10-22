#include "main/main.hpp"

#define DEBUG

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::MultiThreadedSpinner spinner(4); // Reduced to 4 threads

    //---Init
    if (Init() != 0)
    {
        ROS_ERROR("Failed to init vision.");
        return -1;
    }

    //---Timer
    main_tim = nh.createTimer(ros::Duration(1.0 / DESIRED_FREQUENCY), CllbckMain);

    sub_raw_frame = it.subscribe("/vision/raw/frame", 1, CllbckSubRawFrame);

#ifdef DEBUG
    // Start spinner in another thread
    std::thread spinner_thread([&spinner]()
                               { spinner.spin(); });

    // GUI-related tasks on the main thread
    while (ros::ok())
    {
        if (!raw_frame.empty())
        {
            //---Giving the IRIS's format of image processing
            cv::imshow("Frame", display_field);
            cv::waitKey(1);
        }
    }
    spinner_thread.join();
#else
    spinner.spin();
    main_tim.stop();
#endif

    return 0;
}

int Init()
{
    // cv::namedWindow("Field Threshold Control", cv::WINDOW_AUTOSIZE);

    return 0;
}

//---Callback
void CllbckMain(const ros::TimerEvent &event)
{
    if (!raw_frame.empty())
    {
        // Process the frame for field
        mtx_field_frame.lock();
        if (field_threshold[0] > field_threshold[1])
        {
            cv::Mat dst_a, dst_b;
            inRange(raw_frame, cv::Scalar(field_threshold[0], field_threshold[2], field_threshold[4]), cv::Scalar(255, field_threshold[3], field_threshold[5]), dst_a);
            inRange(raw_frame, cv::Scalar(0, field_threshold[2], field_threshold[4]), cv::Scalar(field_threshold[1], field_threshold[3], field_threshold[5]), dst_b);
            bitwise_or(dst_a, dst_b, thresholded_field);
        }
        else
        {
            inRange(raw_frame, cv::Scalar(field_threshold[0], field_threshold[2], field_threshold[4]), cv::Scalar(field_threshold[1], field_threshold[3], field_threshold[5]), thresholded_field);
        }
        mtx_field_frame.unlock();

        ApplyMorphology(thresholded_field, 4);

        cv::Point center = cv::Point(center_cam_x, center_cam_y);
        DrawCenterCamCircle(thresholded_field, center, r_cam, 333, cv::Scalar(255), r_cam);

        raw_thresholded_field = thresholded_field.clone();

        mtx_bgr_field.lock();
        cv::cvtColor(raw_frame, bgr_frame, cv::COLOR_HSV2BGR);
        mtx_bgr_field.unlock();

        mtx_draw_field.lock();
        display_field = bgr_frame.clone();
        mtx_draw_field.unlock();

        std::vector<std::vector<cv::Point>> field_contours;
        std::vector<cv::Vec4i> field_hierarchy;

        cv::findContours(thresholded_field, field_contours, field_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        std::vector<std::vector<cv::Point>> field_poly(field_contours.size());

        for (int i = 0; i < field_contours.size(); i++)
        {
            if (cv::contourArea(field_contours[i]) > 1000)
            {
                // cv::approxPolyDP(cv::Mat(field_contours[i]), field_poly[i], 3, true);
                cv::convexHull(cv::Mat(field_contours[i]), field_poly[i], false);
                cv::drawContours(display_field, field_poly, i, cv::Scalar(0, 255, 0), 2, 8, field_hierarchy, 0, cv::Point());
            }
        }

        //------Ball Part------//
        //=====================//
        }
}

void CllbckSubRawFrame(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        mtx_main_frame.lock();
        raw_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::cvtColor(raw_frame, raw_frame, cv::COLOR_BGR2HSV);
        mtx_main_frame.unlock();
        logger_instance.Log(logger::GREEN, "Frame received.");
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}