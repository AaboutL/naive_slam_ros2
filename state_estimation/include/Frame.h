//
// Created by hanfuyong on 2022/10/17.
//

#ifndef FRAME_H
#define FRAME_H

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__ // 与上一行一样，但是不能删除
#endif

#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

namespace Naive_SLAM_ROS {

class Frame{
public:
    Frame(double timestamp);
    Frame(const Frame& frame);
    Frame(double timestamp, const std::vector<unsigned long>& vChainIds, 
          const std::vector<Eigen::Vector2d>& vPtsUn, const std::vector<Eigen::Vector2d>& vPts, 
          const std::vector<Eigen::Vector2d>& vPtUnOffsets);
        
    void SetTcw(const Eigen::Matrix4d& Tcw);
    void SetTcw(const Eigen::Matrix3d& Rcw, const Eigen::Vector3d& tcw);

public:
    double mdTimestamp;
    std::vector<unsigned long> mvChainIds;
    std::vector<Eigen::Vector2d> mvPtsUn;
    std::vector<Eigen::Vector2d> mvPts;
    std::vector<Eigen::Vector2d> mvPtUnOffsets;

    Eigen::Matrix3d mRcw;
    Eigen::Vector3d mtcw;
    Eigen::Matrix4d mTcw;
    Eigen::Matrix3d mRwc;
    Eigen::Vector3d mtwc;
    Eigen::Matrix4d mTwc; // position in world frame

};
    
} // namespace naive_slam_ros


#endif