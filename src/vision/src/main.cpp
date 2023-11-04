#include "main/main.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::MultiThreadedSpinner spinner(8); // Reduced to 4 threads

    //---Subscribers
    sub_vision_field_threshold_params = nh.subscribe("/vision/field/threshold/params", 1, CllbckSubVisionFieldThresholdParams);
    sub_vision_ball_threshold_params = nh.subscribe("/vision/ball/threshold/params", 1, CllbckSubVisionBallThresholdParams);
    sub_raw_frame = it.subscribe("/vision/raw/frame", 1, CllbckSubRawFrame);

    //---Publishers
    pub_display_frame = it.advertise("/vision/display/frame", 1);
    pub_thresholded_field = it.advertise("/vision/thresholded/field", 1);
    pub_thresholded_ball = it.advertise("/vision/thresholded/ball", 1);
    pub_thresholded_line = it.advertise("/vision/thresholded/line", 1);

    //---Service
    srv_vision_field_threshold_params = nh.advertiseService("/vision/field/threshold/params/srv", SrvVisionFieldThresholdParams);
    srv_vision_ball_threshold_params = nh.advertiseService("/vision/ball/threshold/params/srv", SrvVisionBallThresholdParams);

    //---Timer
    main_tim = nh.createTimer(ros::Duration(1.0 / DESIRED_FREQUENCY), CllbckMain);

    //---Init
    if (Init() != 0)
    {
        ROS_ERROR("Failed to init vision.");
        return -1;
    }

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

    //---Field Part---//
    {
        std::lock_guard<std::mutex> lock(mtx_field_frame);
        if (field_threshold[0] > field_threshold[1])
        {
            cv::Mat dst_a, dst_b;
            cv::inRange(raw_frame, cv::Scalar(field_threshold[0], field_threshold[2], field_threshold[4]), cv::Scalar(255, field_threshold[3], field_threshold[5]), dst_a);
            cv::inRange(raw_frame, cv::Scalar(0, field_threshold[2], field_threshold[4]), cv::Scalar(field_threshold[1], field_threshold[3], field_threshold[5]), dst_b);
            cv::bitwise_or(dst_a, dst_b, thresholded_field);
        }
        else
        {
            cv::inRange(raw_frame, cv::Scalar(field_threshold[0], field_threshold[2], field_threshold[4]), cv::Scalar(field_threshold[1], field_threshold[3], field_threshold[5]), thresholded_field);
        }
    }
    ApplyMorphology(thresholded_field, 4);

#ifdef DEVELOPMENT
    // cv::imshow("thresholded_field", thresholded_field);
    // cv::imshow("raw_frame", raw_frame);
#endif

    cv::Point center = cv::Point(center_cam_x, center_cam_y);
    // DrawCenterCamCircle(thresholded_field, center, r_cam, 333, cv::Scalar(255), r_cam);

    raw_thresholded_field = thresholded_field.clone();

    {
        std::lock_guard<std::mutex> lock(mtx_bgr_field);
        cv::cvtColor(raw_frame, bgr_frame, cv::COLOR_HSV2BGR);
    }

    {
        std::lock_guard<std::mutex> lock(mtx_draw_field);
        display_field = bgr_frame.clone();
    }

    std::vector<std::vector<cv::Point>> field_contours;
    std::vector<cv::Vec4i> field_hierarchy;

    cv::findContours(thresholded_field, field_contours, field_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    std::vector<std::vector<cv::Point>> field_poly(field_contours.size());

    for (int i = 0; i < field_contours.size(); i++)
    {
        if (cv::contourArea(field_contours[i]) > 1000)
        {
            cv::convexHull(cv::Mat(field_contours[i]), field_poly[i], false);
            cv::drawContours(thresholded_field, field_poly, i, cv::Scalar(255), -1, 8, field_hierarchy, 0, cv::Point());
            cv::drawContours(display_field, field_poly, i, cv::Scalar(0, 255, 0), 2, 8, field_hierarchy, 0, cv::Point());
        }
    }

    cv::circle(thresholded_field, cv::Point(center_cam_x, center_cam_y), r_cam, cv::Scalar(0), -1); // exclude center cam

    cv::bitwise_and(raw_thresholded_field, thresholded_field, raw_thresholded_field);

    //------Ball Part------//
    //=====================//
    {
        std::lock_guard<std::mutex> lock(mtx_ball_frame);
        raw_ball = bgr_frame.clone();
        cv::cvtColor(raw_ball, raw_ball, cv::COLOR_BGR2HSV);
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
    }

    ApplyMorphology(thresholded_ball, 5);

    // cv::bitwise_and(thresholded_ball, thresholded_field, thresholded_ball);
    {
        std::lock_guard<std::mutex> lock(mtx_ball_and_field_frame);
        cv::bitwise_and(thresholded_ball, thresholded_field, thresholded_ball);
    }

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

    TransmitFrame();

#ifdef DEVELOPMENT
    // cv::imshow("display_field", display_field);
    cv::waitKey(1);
#endif
}

void CllbckSubRawFrame(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        mtx_main_frame.lock();
        recieved_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::cvtColor(recieved_frame, raw_frame, cv::COLOR_BGR2HSV);
        mtx_main_frame.unlock();
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void CllbckSubVisionFieldThresholdParams(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mtx_field_frame);
    for (int i = 0; i < 6; i++)
    {
        field_threshold[i] = msg->data[i];
    }
}

void CllbckSubVisionBallThresholdParams(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mtx_ball_frame);
    for (int i = 0; i < 6; i++)
    {
        ball_threshold[i] = msg->data[i];
    }
}

bool SrvVisionFieldThresholdParams(msg_collection::ThresholdVision::Request &req, msg_collection::ThresholdVision::Response &res)
{
    try
    {
        for (uint8_t i = 0; i < 6; i++)
            res.threshold_params.push_back(field_threshold[i]);

        logger_instance.Log(logger::GREEN, "Vision field threshold params has been sent.");
    }
    catch (const std::exception &e)
    {
        std::cerr << "Load Config: " << e.what() << '\n';
    }
    return true;
}

bool SrvVisionBallThresholdParams(msg_collection::ThresholdVision::Request &req, msg_collection::ThresholdVision::Response &res)
{
    try
    {
        for (uint8_t i = 0; i < 6; i++)
            res.threshold_params.push_back(ball_threshold[i]);

        logger_instance.Log(logger::GREEN, "Vision field threshold params has been sent.");
    }
    catch (const std::exception &e)
    {
        std::cerr << "Load Config: " << e.what() << '\n';
    }
    return true;
}
void TransmitFrame()
{
    cv::Mat send_buf;

    // Publish display_field image
    send_buf = display_field.clone();
    sensor_msgs::ImagePtr display_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", send_buf).toImageMsg();
    pub_display_frame.publish(display_msg);

    // Publish thresholded_field image
    send_buf = thresholded_field.clone();
    sensor_msgs::ImagePtr thresholded_field_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", send_buf).toImageMsg();
    pub_thresholded_field.publish(thresholded_field_msg);

    // Publish thresholded_ball image
    send_buf = thresholded_ball.clone();
    sensor_msgs::ImagePtr thresholded_ball_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", send_buf).toImageMsg();
    pub_thresholded_ball.publish(thresholded_ball_msg);
}