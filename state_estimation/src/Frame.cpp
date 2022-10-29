//
// Created by hanfuyong on 2022/10/17.
//

#include "Frame.h"

namespace Naive_SLAM_ROS{

Frame::Frame(double timestamp):mdTimestamp(timestamp){
    SetTcw(Eigen::Matrix4d::Identity());
}

Frame::Frame(const Frame& frame):
mdTimestamp(frame.mdTimestamp), mvChainIds(frame.mvChainIds), mvPtsUn(frame.mvPtsUn),
mvPts(frame.mvPts), mvPtUnOffsets(frame.mvPtUnOffsets), mRcw(frame.mRcw), mtcw(frame.mtcw),
mTcw(frame.mTcw), mRwc(frame.mRwc), mtwc(frame.mtwc), mTwc(frame.mTwc){
}

Frame::Frame(double timestamp, const std::vector<unsigned long>& vChainIds, 
             const std::vector<Eigen::Vector2d>& vPtsUn, const std::vector<Eigen::Vector2d>& vPts,
             const std::vector<Eigen::Vector2d>& vPtUnOffsets):
mdTimestamp(timestamp), mvChainIds(vChainIds), mvPtsUn(vPtsUn), mvPts(vPts), mvPtUnOffsets(vPtUnOffsets){
    SetTcw(Eigen::Matrix4d::Identity());
}

void Frame::SetTcw(const Eigen::Matrix4d& Tcw){
    mTcw = Tcw;

    mRcw = Tcw.block<3, 3>(0, 0);
    mtcw = Tcw.block<3, 1>(0, 3);

    mRwc = mRcw.transpose();
    mtwc = -mRwc * mtcw;

    mTwc = Eigen::Matrix4d::Identity();
    mTwc.block<3, 3>(0, 0) = mRwc;
    mTwc.block<3, 1>(0, 3) = mtwc;
}

void Frame::SetTcw(const Eigen::Matrix3d& Rcw, const Eigen::Vector3d& tcw){
    mRcw = Rcw;
    mtcw = tcw;
    mTcw = Eigen::Matrix4d::Identity();
    mTcw.block<3, 3>(0, 0) = mRcw;
    mTcw.block<3, 1>(0, 3) = mtcw;

    mTwc = Eigen::Matrix4d::Identity();
    mRwc = Rcw.transpose();
    mtwc = -mRwc * tcw;
    mTwc.block<3, 3>(0, 0) = mRwc;
    mTwc.block<3, 1>(0, 3) = mtwc;
}

} // namespace naive_slam_ros