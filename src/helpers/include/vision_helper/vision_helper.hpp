#ifndef VISION_HELPER_HPP
#define VISION_HELPER_HPP

#include "opencv2/opencv.hpp"

/**
 * Function to apply morphological transformations
 */
void ApplyMorphology(cv::Mat &src, int size)
{
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size));
    erode(src, src, element);
    dilate(src, src, element);
}

/**
 * Function to draw circle with ignoring the center cam
 */
void DrawCenterCamCircle(cv::Mat &src, cv::Point center, int outer_radius, int inner_radius, cv::Scalar color, int thickness)
{
    cv::circle(src, center, outer_radius, cv::Scalar(255), -1);
    cv::circle(src, center, inner_radius, cv::Scalar(0), thickness);
}

#endif