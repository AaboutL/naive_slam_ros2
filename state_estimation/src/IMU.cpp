//
// Created by hanfuyong on 2022/10/18.
//

#include "IMU.h"

namespace Naive_SLAM_ROS{

IMU::IMU(uint32_t timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr):
miTimestamp(timestamp), mAcc(acc), mGyr(gyr){
}
    
} // namespace Naive_SLAM_ROS
