#ifndef AM_UTILS_AM_DETECTION_H_
#define AM_UTILS_AM_DETECTION_H_

#include <string>
#include <opencv2/opencv.hpp>

namespace am
{
struct Detection
{

    int class_id_{0};
    std::string class_name_;
    double confidence{0.0};
    cv::Scalar color{};
    cv::Rect box{};
};
}

#endif /*AM_UTILS_AM_DETECTION_H_*/