//
// Created by hanfuyong on 2022/12/10
//

#include "utils.h"

namespace Naive_SLAM_ROS{
    void Utils::PrintCvMat(const std::string& msg, const cv::Mat& mat){
        std::cout << msg << std::endl;
        for(int i = 0; i < mat.rows; i++){
            for(int j = 0; j < mat.cols; j++){
                std::cout << mat.at<float>(i, j) << " ";
            }
            std::cout << std::endl;
        }
    }
}