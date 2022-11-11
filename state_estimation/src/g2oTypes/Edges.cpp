//
// Created by hanfuyong on 2022/11/09
//

#include "g2oTypes/Edges.h"

namespace Naive_SLAM_ROS{


EdgeInertialGS::EdgeInertialGS(Preintegrator* pPreint):
mJRbg(pPreint->GetJRbg()), mJVba(pPreint->GetJVba()), mJVbg(pPreint->GetJVbg()), 
mJPba(pPreint->GetJPba()), mJPbg(pPreint->GetJPbg()), mpPreint(pPreint), 
mGI(Eigen::Vector3d(0, 0, -9.81)), mdT(pPreint->GetDeltaT()){
    resize(8); // This edge has 8 vertices
    Matrix9d info = pPreint->GetCov().block<9, 9>(0, 0).inverse();
    info = (info + info.transpose()) * 0.5; // make symmetrical matrix. Infomation matrix is a symmetrical matrix
    Eigen::SelfAdjointEigenSolver<Matrix9d> es(info);
    Vector9d eigs = es.eigenvalues();
    for(int i = 0; i < 9; i++){
        if(eigs[i] < 1e-12)
            eigs[i] = 0;
    }
    info = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
    setInformation(info);
}
    
void EdgeInertialGS::computeError(){
    const VertexPose* VPose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1 = static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyrBias* VGB = static_cast<const VertexGyrBias*>(_vertices[2]);
    const VertexAccBias* VAB = static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VPose2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
    const VertexGDir* VGDir = static_cast<const VertexGDir*>(_vertices[6]);
    const VertexScale* VS = static_cast<const VertexScale*>(_vertices[7]);

    // const IMU::Bias b(VA->estimate()[0],VA->estimate()[1],VA->estimate()[2],VG->estimate()[0],VG->estimate()[1],VG->estimate()[2]);
    const Eigen::Vector3d gyrBiasNew = VGB->estimate();
    const Eigen::Vector3d accBiasNew = VAB->estimate();
    mG = VGDir->estimate().mRwg*mGI;
    const double s = VS->estimate();

    Eigen::Matrix3d dR;
    Eigen::Vector3d dV, dP;
    mpPreint->UpdateDeltaPVR(gyrBiasNew, accBiasNew, dP, dV, dR);

    auto R1wb = VPose1->estimate().mTwb.so3();
    auto t1wb = VPose1->estimate().mTwb.translation();
    auto R2wb = VPose2->estimate().mTwb.so3();
    auto t2wb = VPose2->estimate().mTwb.translation();
    Eigen::Vector3d er = (Sophus::SO3d(dR.transpose()) * R1wb.inverse() * R2wb).log();
    Eigen::Vector3d ev = R1wb.matrix().transpose() * (s *(VV2->estimate() - VV1->estimate()) - mG * mdT) - dV;
    Eigen::Vector3d ep = R1wb.matrix().transpose() * (s *(t2wb - t1wb - VV1->estimate() * mdT) - mG * mdT * mdT * 0.5) - dP;

    // const Eigen::Matrix3d dR = mpInt->GetDeltaRotation(b).cast<double>();
    // const Eigen::Vector3d dV = mpInt->GetDeltaVelocity(b).cast<double>();
    // const Eigen::Vector3d dP = mpInt->GetDeltaPosition(b).cast<double>();

    // const Eigen::Vector3d er = LogSO3(dR.transpose()*VPose1->estimate().Rwb.transpose()*VPose2->estimate().Rwb);
    // const Eigen::Vector3d ev = VPose1->estimate().Rwb.transpose()*(s*(VV2->estimate() - VV1->estimate()) - g*dt) - dV;
    // const Eigen::Vector3d ep = VPose1->estimate().Rwb.transpose()*(s*(VPose2->estimate().twb - VPose1->estimate().twb - VV1->estimate()*dt) - g*dt*dt/2) - dP;

    _error << er, ev, ep;
}

void EdgeInertialGS::linearizeOplus(){
    const VertexPose* VPose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyrBias* VGB= static_cast<const VertexGyrBias*>(_vertices[2]);
    const VertexAccBias* VAB= static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VPose2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
    const VertexGDir* VGDir = static_cast<const VertexGDir*>(_vertices[6]);
    const VertexScale* VS = static_cast<const VertexScale*>(_vertices[7]);

    double s = VS->estimate();

    const Eigen::Vector3d gyrBiasNew = VGB->estimate();
    const Eigen::Vector3d accBiasNew = VAB->estimate();
    Eigen::Vector3d deltaGyrBias = gyrBiasNew - mpPreint->GetGyrBias();

    Eigen::Matrix3d dR;
    Eigen::Vector3d dV, dP;
    mpPreint->UpdateDeltaPVR(gyrBiasNew, accBiasNew, dP, dV, dR); // update DeltaR, DeltaV, DeltaP

    auto R1wb = VPose1->estimate().mTwb.so3();
    auto t1wb = VPose1->estimate().mTwb.translation();
    auto R1bw = R1wb.inverse();

    auto R2wb = VPose2->estimate().mTwb.so3();
    auto t2wb = VPose2->estimate().mTwb.translation();
    auto R2bw = R2wb.inverse();

    auto Rwg = VGDir->estimate().mRwg;
    Eigen::MatrixXd Gm = Eigen::MatrixXd::Zero(3,2);
    Gm(0,1) = -9.81;
    Gm(1,0) = 9.81;
    const Eigen::MatrixXd dGdTheta = Rwg*Gm;

    Eigen::Matrix3d eR = dR.transpose() * (R1bw * R2wb).matrix();
    Eigen::Vector3d er = Sophus::SO3d(eR).log();
    Eigen::Matrix3d invJr = InverseRightJacobian(er);

    // Jacobian(9 x 6) wrt Pose1(include rotation and position) 
    _jacobianOplus[0].setZero();
    // rotation residual wrt Pose1 R
    _jacobianOplus[0].block<3,3>(0, 0) = -invJr * (R2bw * R1wb).matrix();
    // velocity residual wrt Pose1 R
    _jacobianOplus[0].block<3,3>(3, 0) = Sophus::SO3d::hat(R1bw*(s*(VV2->estimate() - VV1->estimate()) - mG*mdT));
    // pos residual wrt Pose1 R
    _jacobianOplus[0].block<3,3>(6, 0) = Sophus::SO3d::hat(R1bw*(s*(t2wb - t1wb - VV1->estimate()*mdT) - 0.5 * mG * mdT * mdT));
    // pos residual wrt Pose1 position
    _jacobianOplus[0].block<3,3>(6, 3) = Eigen::DiagonalMatrix<double, 3>(-s, -s, -s);

    // jacobian(9 x 3) wrt V1
    _jacobianOplus[1].setZero();
    // velocity residual wrt V1
    _jacobianOplus[1].block<3,3>(3, 0) = -s * R1bw.matrix();
    // position residual wrt V1
    _jacobianOplus[1].block<3,3>(6, 0) = -s * R1bw.matrix() * mdT;

    // jacobian(9 x 3) wrt gyr bias
    _jacobianOplus[2].setZero();
    // rotation residual wrt gyr bias
    _jacobianOplus[2].block<3,3>(0,0) = -invJr * eR.transpose() * RightJacobian(mJRbg * deltaGyrBias) * mJRbg;
    // velocity residual wrt gyr bias
    _jacobianOplus[2].block<3,3>(3,0) = -mJVbg;
    // position residual wrt gyr bias
    _jacobianOplus[2].block<3,3>(6,0) = -mJPbg;

    // jacobian(9 x 3) wrt acc bias
    _jacobianOplus[3].setZero();
    // velocity residual wrt acc bias
    _jacobianOplus[3].block<3,3>(3,0) = -mJVba;
    // position residual wrt acc bias
    _jacobianOplus[3].block<3,3>(6,0) = -mJPba;

    // Jacobian(9 x 6) wrt Pose2(include rotation and position) 
    _jacobianOplus[4].setZero();
    // rotation residual wrt Pose2 R
    _jacobianOplus[4].block<3,3>(0, 0) = invJr;
    // position residual wrt Pose2 position
    _jacobianOplus[4].block<3,3>(6,3) = s * (R1bw * R2wb).matrix();

    // jacobian(9 x 3) wrt V2
    _jacobianOplus[5].setZero();
    // velocity residual wrt V2
    _jacobianOplus[5].block<3,3>(3,0) = s * R1bw.matrix();

    // jacobian(9 x 2) wrt Gdir
    _jacobianOplus[6].setZero();
    // velocity residual wrt gdir
    _jacobianOplus[6].block<3,2>(3,0) = -R1bw.matrix() * dGdTheta * mdT;
    // position residual wrt gdir
    _jacobianOplus[6].block<3,2>(6,0) = -0.5 * R1bw.matrix() * dGdTheta * mdT * mdT;

    // jacobian(9, 1) wrt scale
    _jacobianOplus[7].setZero();
    // velocity residual wrt scale
    _jacobianOplus[7].block<3,1>(3,0) = R1bw.matrix() * (VV2->estimate() - VV1->estimate()) * s;
    // position residual wrt scale
    _jacobianOplus[7].block<3,1>(6,0) = R1bw.matrix() * (t2wb - t1wb - VV1->estimate() * mdT) * s;
}


} // namespace Naive_SLAM_ROS