//
// Created by hanfuyong on 2022/10/17.
//

#ifndef FRAME_H
#define FRAME_H

#include <iostream>
#include <opencv2/opencv.hpp>

namespace Naive_SLAM_ROS {

class Frame{
public:
    Frame(uint32_t timestamp);
    Frame(uint32_t timestamp, const std::vector<long unsigned int>& vChainIds, 
          const std::vector<cv::Vec3f>& vPtsUn, const std::vector<cv::Vec2f>& vPts, 
          const std::vector<cv::Vec2f>& vPtUnOffsets);

public:
    uint32_t miTimestamp;
    std::vector<long unsigned int> mvChainIds;
    std::vector<cv::Vec3f> mvPtsUn;
    std::vector<cv::Vec2f> mvPts;
    std::vector<cv::Vec2f> mvPtUnOffsets;
};
    
} // namespace naive_slam_ros


#endif