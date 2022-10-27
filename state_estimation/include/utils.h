//
// Created by hanfuyong on 2022/10/26
//

#ifndef NAIVESLAMROS_UTILS_H
#define NAIVESLAMROS_UTILS_H

#include <opencv2/opencv.hpp>

namespace Naive_SLAM_ROS{

    void PrintCvMat(const std::string& msg, const cv::Mat& mat){
        std::cout << msg << std::endl;
        for(int i = 0; i < mat.rows; i++){
            for(int j = 0; j < mat.cols; j++){
                std::cout << mat.at<float>(i, j) << " ";
            }
            std::cout << std::endl;
        }
    }
    
} // namespace Naive_SLAM_ROS


#endif