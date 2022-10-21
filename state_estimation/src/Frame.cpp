//
// Created by hanfuyong on 2022/10/17.
//

#include "Frame.h"

namespace Naive_SLAM_ROS{

Frame::Frame(uint32_t timestamp):miTimestamp(timestamp)
{}

Frame::Frame(uint32_t timestamp, const std::vector<long unsigned int>& vChainIds, 
             const std::vector<cv::Vec3f>& vPtsUn, const std::vector<cv::Vec2f>& vPts,
             const std::vector<cv::Vec2f>& vPtUnOffsets):
miTimestamp(timestamp), mvChainIds(vChainIds), mvPtsUn(vPtsUn), mvPts(vPts), mvPtUnOffsets(vPtUnOffsets){
}

} // namespace naive_slam_ros