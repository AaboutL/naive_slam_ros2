//
// Created by hanfuyong on 2022/10/18
//

#ifndef IMU_H
#define IMU_H

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__ // same but can not remove
#endif

#include <iostream>
#include <eigen3/Eigen/Dense>

namespace Naive_SLAM_ROS
{

class IMU{
public:
    IMU(uint32_t timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr);
public:
    uint32_t miTimestamp;
    Eigen::Vector3d mAcc;
    Eigen::Vector3d mGyr;
    Eigen::Vector3d mVel;
    Eigen::Vector3d mAcc_b;
    Eigen::Vector3d mGry_b;

};

    
} // namespace Naive_SLAM_ROS


#endif