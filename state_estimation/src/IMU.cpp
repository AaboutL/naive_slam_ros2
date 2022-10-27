//
// Created by hanfuyong on 2022/10/18.
//

#include "IMU.h"

namespace Naive_SLAM_ROS{

IMU::IMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr):
mdTimestamp(timestamp), mAcc(acc), mGyr(gyr){
}
    
} // namespace Naive_SLAM_ROS
