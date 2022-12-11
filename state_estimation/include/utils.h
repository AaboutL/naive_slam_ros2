//
// Created by hanfuyong on 2022/10/26
//

#ifndef NAIVESLAMROS_UTILS_H
#define NAIVESLAMROS_UTILS_H

#include <opencv2/opencv.hpp>

namespace Naive_SLAM_ROS{
class Utils{
public:
    static void PrintCvMat(const std::string& msg, const cv::Mat& mat);
    // static void WriteFile(const std::string& ts1, const std::string& ts2,
    //                       const std::vector<cv::Point2)
};
    
} // namespace Naive_SLAM_ROS


#endif