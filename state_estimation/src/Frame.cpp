//
// Created by hanfuyong on 2022/10/17.
//

#include "Frame.h"

namespace Naive_SLAM_ROS{

Frame::Frame(double timestamp):mdTimestamp(timestamp){
    SetTcw(cv::Mat::eye(4, 4, CV_32F));
    
}

Frame::Frame(const Frame& frame):
mdTimestamp(frame.mdTimestamp), mvChainIds(frame.mvChainIds), mvPtsUn(frame.mvPtsUn),
mvPts(frame.mvPts), mvPtUnOffsets(frame.mvPtUnOffsets), mRcw(frame.mRcw.clone()), mtcw(frame.mtcw.clone()),
mTcw(frame.mTcw.clone()), mRwc(frame.mRwc.clone()), mtwc(frame.mtwc.clone()), mTwc(frame.mTwc.clone()){

}

Frame::Frame(double timestamp, const std::vector<unsigned long>& vChainIds, 
             const std::vector<cv::Vec3f>& vPtsUn, const std::vector<cv::Vec2f>& vPts,
             const std::vector<cv::Vec2f>& vPtUnOffsets):
mdTimestamp(timestamp), mvChainIds(vChainIds), mvPtsUn(vPtsUn), mvPts(vPts), mvPtUnOffsets(vPtUnOffsets){
    SetTcw(cv::Mat::eye(4, 4, CV_32F));
}

void Frame::SetTcw(const cv::Mat& Tcw){
    mTcw = Tcw.clone();

    mRcw = Tcw.rowRange(0, 3).colRange(0, 3);
    mtcw = Tcw.rowRange(0, 3).col(3);

    mRwc = mRcw.t();
    mtwc = -mRwc * mtcw;

    mTwc = cv::Mat::eye(4, 4, CV_32F);
    mRwc.copyTo(mTwc.rowRange(0, 3).colRange(0, 3));
    mtwc.copyTo(mTwc.rowRange(0, 3).col(3));
}

void Frame::SetTcw(const cv::Mat& Rcw, const cv::Mat& tcw){
    mRcw = Rcw;
    mtcw = tcw;
    mTcw = cv::Mat::eye(4, 4, CV_32F);
    mRcw.copyTo(mTcw.rowRange(0, 3).colRange(0, 3));
    mtcw.copyTo(mTcw.rowRange(0, 3).col(3));

    mTwc = cv::Mat::eye(4, 4, CV_32F);
    mRwc = Rcw.t();
    mtwc = -Rcw.t() * tcw;
    mRwc.copyTo(mTwc.rowRange(0, 3).colRange(0, 3));
    mtwc.copyTo(mTwc.rowRange(0, 3).col(3));
}

} // namespace naive_slam_ros