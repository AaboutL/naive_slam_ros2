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
    Frame(double timestamp);
    Frame(const Frame& frame);
    Frame(double timestamp, const std::vector<unsigned long>& vChainIds, 
          const std::vector<cv::Vec3f>& vPtsUn, const std::vector<cv::Vec2f>& vPts, 
          const std::vector<cv::Vec2f>& vPtUnOffsets);
        
    void SetTcw(const cv::Mat& Tcw);
    void SetTcw(const cv::Mat& Rcw, const cv::Mat& tcw);

public:
    double mdTimestamp;
    std::vector<unsigned long> mvChainIds;
    std::vector<cv::Vec3f> mvPtsUn;
    std::vector<cv::Vec2f> mvPts;
    std::vector<cv::Vec2f> mvPtUnOffsets;

    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mTcw;
    cv::Mat mRwc;
    cv::Mat mtwc;
    cv::Mat mTwc; // position in world frame

};
    
} // namespace naive_slam_ros


#endif