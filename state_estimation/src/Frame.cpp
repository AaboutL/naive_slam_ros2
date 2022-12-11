//
// Created by hanfuyong on 2022/10/17.
//

#include "Frame.h"

namespace Naive_SLAM_ROS{

PointCloud::PointCloud(double timestamp, const std::vector<unsigned long>& vChainIds, 
          const std::vector<Eigen::Vector2d>& vPtsUn, const std::vector<Eigen::Vector2d>& vPts, 
          const std::vector<Eigen::Vector2d>& vPtUnOffsets):
mdTimestamp(timestamp), mvChainIds(vChainIds), mvPtsUn(vPtsUn), mvPts(vPts), mvPtUnOffsets(vPtUnOffsets){
}

PointCloud::PointCloud(double timestamp, const std::string& sTimestamp, const std::vector<unsigned long>& vChainIds, 
          const std::vector<Eigen::Vector2d>& vPtsUn, const std::vector<Eigen::Vector2d>& vPts, 
          const std::vector<Eigen::Vector2d>& vPtUnOffsets):
mdTimestamp(timestamp), mvChainIds(vChainIds), mvPtsUn(vPtsUn), mvPts(vPts), mvPtUnOffsets(vPtUnOffsets),
msTimestamp(sTimestamp){
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Frame::Frame(double timestamp):mdTimestamp(timestamp){
    SetTcw(Sophus::SE3d());
    mVelocity.setZero();
}

Frame::Frame(const Frame& frame):
mdTimestamp(frame.mdTimestamp), mvChainIds(frame.mvChainIds), mvPtsUn(frame.mvPtsUn),
mvPts(frame.mvPts), mvPtUnOffsets(frame.mvPtUnOffsets), mRcw(frame.mRcw), mtcw(frame.mtcw),
mTcw(frame.mTcw), mRwc(frame.mRwc), mtwc(frame.mtwc), mTwc(frame.mTwc), 
mpPreintegrator(frame.mpPreintegrator), mVelocity(frame.mVelocity){
}

Frame::Frame(const Frame* pF):
mdTimestamp(pF->mdTimestamp), mvChainIds(pF->mvChainIds), mvPtsUn(pF->mvPtsUn),
mvPts(pF->mvPts), mvPtUnOffsets(pF->mvPtUnOffsets), mRcw(pF->mRcw), mtcw(pF->mtcw),
mTcw(pF->mTcw), mRwc(pF->mRwc), mtwc(pF->mtwc), mTwc(pF->mTwc), 
mpPreintegrator(pF->mpPreintegrator), mVelocity(pF->mVelocity){
}

Frame::Frame(const PointCloud& pc, const Sophus::SE3d& Tbc):
mdTimestamp(pc.mdTimestamp), msTimestamp(pc.msTimestamp),
mvChainIds(pc.mvChainIds), mvPtsUn(pc.mvPtsUn), mvPts(pc.mvPts), mvPtUnOffsets(pc.mvPtUnOffsets){

    SetTcw(Sophus::SE3d());

    mTbc = Tbc;
    mTcb = Tbc.inverse();

    mpPreintegrator = nullptr;
    mVelocity.setZero();
}

Frame::~Frame(){
    delete mpPreintegrator;
    mpPreintegrator = nullptr;
}

void Frame::SetTcw(const Eigen::Matrix4d& Tcw){
    mTcw = Sophus::SE3d(Tcw);
    mRcw = mTcw.rotationMatrix();
    mtcw = mTcw.translation();

    mTwc = mTcw.inverse();
    mRwc = mTwc.rotationMatrix();
    mtwc = mTwc.translation();
}

void Frame::SetTcw(const Sophus::SE3d& Tcw){
    mTcw = Tcw;
    mRcw = mTcw.rotationMatrix();
    mtcw = mTcw.translation();

    mTwc = mTcw.inverse();
    mRwc = mTwc.rotationMatrix();
    mtwc = mTwc.translation();;
}

void Frame::SetTcw(const Eigen::Matrix3d& Rcw, const Eigen::Vector3d& tcw){
    mRcw = Rcw;
    mtcw = tcw;
    mTcw = Sophus::SE3d(mRcw, mtcw);

    mTwc = mTcw.inverse();
    mRwc = mTwc.rotationMatrix();
    mtwc = mTwc.translation();;
}

void Frame::SetPreintegrator(Preintegrator* pPreintegrator){
    mpPreintegrator = pPreintegrator;
}

void Frame::SetVelocity(const Eigen::Vector3d& vel){
    mVelocity = vel;
}

Eigen::Vector3d Frame::GetBodyPosition() const{
    return mRwc * mTcb.translation() + mtwc;
}

Sophus::SE3d Frame::GetTbc() const{
    return mTbc;
}

Sophus::SE3d Frame::GetTcb() const{
    return mTcb;
}

Sophus::SE3d Frame::GetTcw() const{
    return mTcw;
}

Sophus::SE3d Frame::GetTwc() const{
    return mTwc;
}

Eigen::Vector3d Frame::GetVelocity() const{
    return mVelocity;
}

Eigen::Vector3d Frame::GetGyrBias() const{
    return mpPreintegrator->GetGyrBias();
}

Eigen::Vector3d Frame::GetAccBias() const{
    return mpPreintegrator->GetAccBias();
}

} // namespace naive_slam_ros