//
// Created by hanfuyong on 2022/11/08
//

#include "g2oTypes/Vertices.h"

namespace Naive_SLAM_ROS
{

ImuCamPose::ImuCamPose(const Sophus::SE3d& Tcb, const Sophus::SE3d& Tcw){
    mTcb = Tcb;
    mTbc = mTcb.inverse();
    mTcw = Tcw;
    mTwb = mTcw.inverse() * mTcb;
    its = 0;
}

void ImuCamPose::Update(const double* pdelta){
    Eigen::Vector3d rdelta, tdelta;
    rdelta << pdelta[0], pdelta[1], pdelta[2];
    tdelta << pdelta[3], pdelta[4], pdelta[5];

    auto deltaR = LieAlg::Exp(rdelta);
    deltaR = NormalizeRotation(deltaR);
    
    // Sophus::SE3d deltaT(LieAlg::Exp(rdelta), tdelta);
    Sophus::SE3d deltaT(deltaR, tdelta);
    // Sophus::SE3d deltaT(Sophus::SO3d::exp(rdelta), tdelta);

    mTwb = mTwb * deltaT;

    mTcw = mTcb * mTwb.inverse();
}
    
} // namespace Naive_SLAM_ROS
