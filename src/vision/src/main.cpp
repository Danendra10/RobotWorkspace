#include "main/main.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::MultiThreadedSpinner spinner(8); // Reduced to 4 threads

    //---Init
    if (Init() != 0)
    {
        ROS_ERROR("Failed to init vision.");
        return -1;
    }

    sub_raw_frame = it.subscribe("/vision/raw/frame", 1, CllbckSubRawFrame);

    //---Timer
    main_tim = nh.createTimer(ros::Duration(1.0 / DESIRED_FREQUENCY), CllbckMain);

    spinner.spin();
    main_tim.stop();

    return 0;
}

int Init()
{
    return 0;
}

//---Callback
void CllbckMain(const ros::TimerEvent &event)
{
    if (raw_frame.empty())
    {
        logger_instance.Log(logger::RED, "Frame is empty.");
        return;
    }
    // {
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
    mtx_ball_frame.lock();
    raw_ball = raw_frame.clone();
    if (ball_threshold[0] > ball_threshold[1])
    {
        cv::Mat dst_a, dst_b;
        inRange(raw_ball, cv::Scalar(ball_threshold[0], ball_threshold[2], ball_threshold[4]), cv::Scalar(255, ball_threshold[3], ball_threshold[5]), dst_a);
        inRange(raw_ball, cv::Scalar(0, ball_threshold[2], ball_threshold[4]), cv::Scalar(ball_threshold[1], ball_threshold[3], ball_threshold[5]), dst_b);
        bitwise_or(dst_a, dst_b, thresholded_ball);
    }
    else
    {
        inRange(raw_ball, cv::Scalar(ball_threshold[0], ball_threshold[2], ball_threshold[4]), cv::Scalar(ball_threshold[1], ball_threshold[3], ball_threshold[5]), thresholded_ball);
    }
    mtx_ball_frame.unlock();

    ApplyMorphology(thresholded_ball, 4);

    mtx_ball_and_field_frame.lock();
    cv::bitwise_and(thresholded_ball, raw_thresholded_field, thresholded_ball);
    mtx_ball_and_field_frame.unlock();

    std::vector<std::vector<cv::Point>> ball_contours;
    std::vector<cv::Vec4i> ball_hierarchy;

    cv::findContours(thresholded_ball, ball_contours, ball_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    std::vector<std::vector<cv::Point>> ball_poly(ball_contours.size());

    for (int i = 0; i < ball_contours.size(); i++)
    {
        if (cv::contourArea(ball_contours[i]) > 1000)
        {
            // cv::approxPolyDP(cv::Mat(ball_contours[i]), ball_poly[i], 3, true);
            cv::convexHull(cv::Mat(ball_contours[i]), ball_poly[i], false);
            cv::drawContours(display_field, ball_poly, i, cv::Scalar(0, 0, 255), 2, 8, ball_hierarchy, 0, cv::Point());
        }
    }

    //---Ball Detected Logic
    if (ball_contours.size())
    {
        counter_ball_in += 17;
        counter_ball_out = 0;
    }
    //---Lost the ball
    //================
    else
    {
        counter_ball_in = 0;
        counter_ball_out += 17;
    }

    //---Found The ball
    //=================
    if (counter_ball_in > 100)
        ball_status = BallStatus::FOUND;
    //---Lost the ball
    //================
    else if (counter_ball_out > 100)
        ball_status = BallStatus::NOT_FOUND;

    static cv::Point2f ball_center;
    static float ball_radius;

    if (ball_status == BallStatus::FOUND)
    {
        static cv::Point2f last_ball_detected;

        if (ball_contours.empty())
        {
            ball_center = last_ball_detected;
        }
        else
        {
            cv::minEnclosingCircle(ball_contours[0], ball_center, ball_radius);
            last_ball_detected = ball_center;
        }

        //---Ball position relative to the center of the camera
        //====================================================
        ball_on_frame.pose_x = ball_center.x - center_cam_x;
        ball_on_frame.pose_y = ball_center.y - center_cam_y;
        ball_on_frame.pose_th = atan2(ball_on_frame.pose_y, ball_on_frame.pose_x);

        ball_on_frame.dist = sqrt(pow(ball_on_frame.pose_x, 2) + pow(ball_on_frame.pose_y, 2));
        float th_ball_to_center = ball_on_frame.pose_th - 180;
        while (th_ball_to_center < -180)
            th_ball_to_center += 360;
        while (th_ball_to_center > 180)
            th_ball_to_center -= 360;
        uint pixel = 0;

        int pixel_x = ball_center.x;
        int pixel_y = ball_center.y;
        while (pixel_x >= 0 && pixel_y >= 0 && pixel_x < res_x && pixel_y < res_y)
        {
            if (raw_ball.at<unsigned char>(cv::Point(pixel_x, pixel_y)) != 255)
            {
                ball_on_frame.dist -= pixel;
                break;
            }

            pixel++;
            pixel_x = ball_center.x + pixel * cos(th_ball_to_center * DEG2RAD);
            pixel_y = ball_center.y - pixel * sin(th_ball_to_center * DEG2RAD);
        }

        ball_on_field.pose_th = ball_on_frame.pose_th + robot_on_field.pose_th - 90;
        while (ball_on_field.pose_th < 0)
            ball_on_field.pose_th += 360;
        while (ball_on_field.pose_th > 360)
            ball_on_field.pose_th -= 360;

        // TODO: THE LUT
    }
    else
    {
        ball_on_field.pose_x = 9999;
        ball_on_field.pose_y = 9999;
        ball_on_field.pose_th = 9999;
        ball_on_field.dist = 9999;
    }
    // }
}

void CllbckSubRawFrame(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        mtx_main_frame.lock();
        recieved_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::cvtColor(recieved_frame, raw_frame, cv::COLOR_BGR2HSV);
        mtx_main_frame.unlock();
        logger_instance.Log(logger::GREEN, "Frame received.");
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}