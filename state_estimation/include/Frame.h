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
#include <sophus/se3.hpp>

#include "IMU.h"

namespace Naive_SLAM_ROS {

class PointCloud{
public:
    PointCloud(double timestamp, const std::vector<unsigned long>& vChainIds, 
          const std::vector<Eigen::Vector2d>& vPtsUn, const std::vector<Eigen::Vector2d>& vPts, 
          const std::vector<Eigen::Vector2d>& vPtUnOffsets);

    double mdTimestamp;
    std::vector<unsigned long> mvChainIds;
    std::vector<Eigen::Vector2d> mvPtsUn;
    std::vector<Eigen::Vector2d> mvPts;
    std::vector<Eigen::Vector2d> mvPtUnOffsets;
};

class Frame{
public:
    Frame(double timestamp);
    Frame(const Frame& frame);
    Frame(const Frame* pF);
    Frame(const PointCloud& pc, const Sophus::SE3d& Tbc);
    virtual ~Frame();
        
    void SetTcw(const Eigen::Matrix4d& Tcw);
    void SetTcw(const Sophus::SE3d& Tcw);
    void SetTcw(const Eigen::Matrix3d& Rcw, const Eigen::Vector3d& tcw);
    
    void SetPreintegrationData();
    void SetPreintegrator(Preintegrator* pPreintegrator);

    void SetVelocity(const Eigen::Vector3d& vel);

    Eigen::Vector3d GetBodyPosition() const;
    Sophus::SE3d GetTbc() const;
    Sophus::SE3d GetTcw() const;
    Eigen::Vector3d GetVelocity() const;
    Eigen::Vector3d GetGyrBias() const;
    Eigen::Vector3d GetAccBias() const;

public:
    double mdTimestamp;
    std::vector<unsigned long> mvChainIds;
    std::vector<Eigen::Vector2d> mvPtsUn;
    std::vector<Eigen::Vector2d> mvPts;
    std::vector<Eigen::Vector2d> mvPtUnOffsets;

    Eigen::Matrix3d mRcw;
    Eigen::Vector3d mtcw;
    Sophus::SE3d mTcw;
    Eigen::Matrix3d mRwc;
    Eigen::Vector3d mtwc; // position in world frame
    Sophus::SE3d mTwc; 

    Sophus::SE3d mTbc;
    Sophus::SE3d mTcb;

    Preintegrator* mpPreintegrator;

    Eigen::Vector3d mVelocity;

};
    
} // namespace naive_slam_ros


#endif