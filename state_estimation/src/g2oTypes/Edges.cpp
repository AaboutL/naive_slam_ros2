//
// Created by hanfuyong on 2022/11/09
//

#include "g2oTypes/Edges.h"

namespace Naive_SLAM_ROS{

EdgeInertial::EdgeInertial(Preintegrator* pPreint):
mJRbg(pPreint->GetJRbg()), mJVba(pPreint->GetJVba()), mJVbg(pPreint->GetJVbg()), 
mJPba(pPreint->GetJPba()), mJPbg(pPreint->GetJPbg()), mpPreint(pPreint), 
mG(Eigen::Vector3d(0, 0, -9.81)), mdT(pPreint->GetDeltaT()){
    resize(6); // This edge has 8 vertices
    Matrix9d info = pPreint->GetCov().block<9, 9>(0, 0).inverse();
    // std::cout << "[EdgeInertial::EdgeInertial] Cov: " << std::endl << pPreint->GetCov().block<9, 9>(0, 0) << std::endl;
    info = (info + info.transpose()) * 0.5; // make symmetrical matrix. Infomation matrix is a symmetrical matrix
    Eigen::SelfAdjointEigenSolver<Matrix9d> es(info);
    Vector9d eigs = es.eigenvalues();
    // std::cout << "[EdgeInertial::EdgeInertial] eigs: " << std::endl << eigs.transpose() << std::endl;
    for(int i = 0; i < 9; i++){
        if(eigs[i] < 1e-12)
            eigs[i] = 0;
    }
    info = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
    // std::cout << "[EdgeInertial::EdgeInertial] Info after: " << std::endl << info << std::endl;
    setInformation(info);
}

void EdgeInertial::computeError(){
    const VertexPose* VPose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1 = static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyrBias* VGB = static_cast<const VertexGyrBias*>(_vertices[2]);
    const VertexAccBias* VAB = static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VPose2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);

    const Eigen::Vector3d gyrBiasNew = VGB->estimate();
    const Eigen::Vector3d accBiasNew = VAB->estimate();

    Eigen::Matrix3d dR;
    Eigen::Vector3d dV, dP;
    mpPreint->UpdateDeltaPVR(gyrBiasNew, accBiasNew, dP, dV, dR);

    auto R1wb = VPose1->estimate().mTwb.so3();
    auto t1wb = VPose1->estimate().mTwb.translation();
    auto R2wb = VPose2->estimate().mTwb.so3();
    auto t2wb = VPose2->estimate().mTwb.translation();
    Eigen::Vector3d er = (Sophus::SO3d(dR.transpose()) * R1wb.inverse() * R2wb).log();
    Eigen::Vector3d ev = R1wb.matrix().transpose() * (VV2->estimate() - VV1->estimate() - mG * mdT) - dV;
    Eigen::Vector3d ep = R1wb.matrix().transpose() * (t2wb - t1wb - VV1->estimate() * mdT - mG * mdT * mdT * 0.5) - dP;

    _error << er, ev, ep;
}

void EdgeInertial::linearizeOplus(){
    const VertexPose* VPose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyrBias* VGB= static_cast<const VertexGyrBias*>(_vertices[2]);
    const VertexAccBias* VAB= static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VPose2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);

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

    Eigen::Matrix3d eR = dR.transpose() * (R1bw * R2wb).matrix();
    Eigen::Vector3d er = Sophus::SO3d(eR).log();
    Eigen::Matrix3d invJr = InverseRightJacobian(er);

    // Jacobian(9 x 6) wrt Pose1(include rotation and position) 
    _jacobianOplus[0].setZero();
    // rotation residual wrt Pose1 R
    _jacobianOplus[0].block<3,3>(0, 0) = -invJr * (R2bw * R1wb).matrix();
    // velocity residual wrt Pose1 R
    _jacobianOplus[0].block<3,3>(3, 0) = Sophus::SO3d::hat(R1bw*(VV2->estimate() - VV1->estimate() - mG*mdT));
    // pos residual wrt Pose1 R
    _jacobianOplus[0].block<3,3>(6, 0) = Sophus::SO3d::hat(R1bw*(t2wb - t1wb - VV1->estimate()*mdT - 0.5 * mG * mdT * mdT));
    // pos residual wrt Pose1 position
    _jacobianOplus[0].block<3,3>(6, 3) = Eigen::DiagonalMatrix<double, 3>(-1, -1, -1);

    // jacobian(9 x 3) wrt V1
    _jacobianOplus[1].setZero();
    // velocity residual wrt V1
    _jacobianOplus[1].block<3,3>(3, 0) = -R1bw.matrix();
    // position residual wrt V1
    _jacobianOplus[1].block<3,3>(6, 0) = -R1bw.matrix() * mdT;

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
    _jacobianOplus[4].block<3,3>(6,3) = (R1bw * R2wb).matrix();

    // jacobian(9 x 3) wrt V2
    _jacobianOplus[5].setZero();
    // velocity residual wrt V2
    _jacobianOplus[5].block<3,3>(3,0) = R1bw.matrix();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

EdgeInertialGS::EdgeInertialGS(Preintegrator* pPreint):
mJRbg(pPreint->GetJRbg()), mJVba(pPreint->GetJVba()), mJVbg(pPreint->GetJVbg()), 
mJPba(pPreint->GetJPba()), mJPbg(pPreint->GetJPbg()), mpPreint(pPreint), 
mGI(Eigen::Vector3d(0, 0, -9.81)), mdT(pPreint->GetDeltaT()){
    resize(8); // This edge has 8 vertices
    Matrix9d info = pPreint->GetCov().block<9, 9>(0, 0).inverse();
    // std::cout << "[EdgeInertialGS::EdgeInertialGS] Cov: " << std::endl << pPreint->GetCov().block<9, 9>(0, 0) << std::endl;
    info = (info + info.transpose()) * 0.5; // make symmetrical matrix. Infomation matrix is a symmetrical matrix
    Eigen::SelfAdjointEigenSolver<Matrix9d> es(info);
    Vector9d eigs = es.eigenvalues();
    // std::cout << "[EdgeInertialGS::EdgeInertialGS] eigs: " << std::endl << eigs.transpose() << std::endl;
    for(int i = 0; i < 9; i++){
        if(eigs[i] < 1e-12)
            eigs[i] = 0;
    }
    info = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
    // std::cout << "[EdgeInertialGS::EdgeInertialGS] Info after: " << std::endl << info << std::endl;
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
    _jacobianOplus[7].block<3,1>(3,0) = R1bw.matrix() * (VV2->estimate() - VV1->estimate());
    // position residual wrt scale
    _jacobianOplus[7].block<3,1>(6,0) = R1bw.matrix() * (t2wb - t1wb - VV1->estimate() * mdT);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

g2o::Vector2 EdgeMono::cam_project(const g2o::Vector3 &trans_xyz) const {
    g2o::Vector2 proj = g2o::project(trans_xyz);
    g2o::Vector2 res;
    res[0] = proj[0] * fx + cx;
    res[1] = proj[1] * fy + cy;
    return res;
}

void EdgeMono::computeError()
{
    const g2o::VertexPointXYZ *vPoint = static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);
    const VertexPose *vPose = static_cast<const VertexPose *>(_vertices[1]);
    const Eigen::Vector2d obs(_measurement);
    _error = obs - cam_project(vPose->estimate().mTcw * vPoint->estimate());
    // std::cout << "_error: " << _error.transpose() << std::endl;
}

void EdgeMono::linearizeOplus(){
    auto* vPoint = static_cast<g2o::VertexPointXYZ*>(_vertices[0]);
    auto* vPose = static_cast<VertexPose*>(_vertices[1]);

    auto Tcw = vPose->estimate().mTcw;
    auto Tbc = vPose->estimate().mTbc;
    auto Rcb = vPose->estimate().mTcb.rotationMatrix();
    auto Pc = Tcw * vPoint->estimate();
    auto Pb = Tbc * Pc;

    // jacobian of residual wrt Point in cam
    Eigen::Matrix<double, 2, 3> JProjPc;
    JProjPc(0, 0) = fx / Pc[2];
    JProjPc(0, 1) = 0;
    JProjPc(0, 2) = -fx * Pc[0] / (Pc[2] * Pc[2]);
    JProjPc(1, 0) = 0;
    JProjPc(1, 1) = fy / Pc[2];
    JProjPc(1, 2) = -fy * Pc[1] / (Pc[2] * Pc[2]);

    // jacobian of residual wrt Point in world
    _jacobianOplusXi = -JProjPc * Tcw.rotationMatrix();

    // jacobian of Pb wrt imu pose
    Eigen::Matrix<double, 3, 6> JPose;
    JPose <<    0,     Pb[2],   -Pb[1],  1, 0, 0,
             -Pb[2],    0,       Pb[0],  0, 1, 0,
              Pb[1],  -Pb[0],      0,    0, 0, 1;

    _jacobianOplusXj = JProjPc * Rcb * JPose;
}

} // namespace Naive_SLAM_ROS
