//
// Created by hanfuyong on 2022/10/18.
//

#include "IMU.h"

namespace Naive_SLAM_ROS{

IMU::IMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr, double dt):
mdTimestamp(timestamp), mAcc(acc), mGyr(gyr), mVel(Eigen::Vector3d::Zero()), mdt(dt){
}

IMU::IMU(const IMU& imu):
mdTimestamp(imu.mdTimestamp), mAcc(imu.mAcc), mGyr(imu.mGyr), mVel(imu.mVel), mdt(imu.mdt), msTimestamp(imu.msTimestamp){}

IMU::IMU(double timestamp, const std::string& sTimestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr, double dt):
mdTimestamp(timestamp), mAcc(acc), mGyr(gyr), mVel(Eigen::Vector3d::Zero()), mdt(dt), msTimestamp(sTimestamp){
}

////////////////////////////////////////////////////////////////////////////////////////////////////

Preintegrator::Preintegrator(const Eigen::DiagonalMatrix<double, 6>& noiseGyrAcc,
                             const Eigen::DiagonalMatrix<double, 6>& noiseGyrAccWalk,
                             const Eigen::Vector3d& gyrBias, const Eigen::Vector3d& accBias):
mGyrAccNoise(noiseGyrAcc), mGyrAccBiasWalk(noiseGyrAccWalk),
mGyrBias(gyrBias), mAccBias(accBias){
    mDeltaR.setIdentity();
    mDeltaV.setZero();
    mDeltaP.setZero();
    mCov.setZero();
    mJacobian.setIdentity();
    mJRbg.setZero();
    mJVba.setZero();
    mJVbg.setZero();
    mJPba.setZero();
    mJPbg.setZero();
    mDeltaGyrBias.setZero();
    mDeltaAccBias.setZero();
    mvIMUMeas.clear();
    mdT = 0;
}

Preintegrator::Preintegrator(double gyrNoise, double accNoise, double gyrBiasWalk, double accBiasWalk,
                             const Eigen::Vector3d& gyrBias, const Eigen::Vector3d& accBias):
mGyrBias(gyrBias), mAccBias(accBias){
    std::cout << "[Preintegrator::Preintegrator] Start" << std::endl;
    mDeltaR.setIdentity();
    mDeltaV.setZero();
    mDeltaP.setZero();
    mCov.setZero();
    mJacobian.setIdentity();
    mJRbg.setZero();
    mJVba.setZero();
    mJVbg.setZero();
    mJPba.setZero();
    mJPbg.setZero();

    mGyrAccNoise.diagonal() << gyrNoise*gyrNoise, gyrNoise*gyrNoise, gyrNoise*gyrNoise,
                               accNoise*accNoise, accNoise*accNoise, accNoise*accNoise;
    mGyrAccBiasWalk.diagonal() << gyrBiasWalk*gyrBiasWalk, gyrBiasWalk*gyrBiasWalk, gyrBiasWalk*gyrBiasWalk,
                                  accBiasWalk*accBiasWalk, accBiasWalk*accBiasWalk, accBiasWalk*accBiasWalk;

    mDeltaGyrBias.setZero();
    mDeltaAccBias.setZero();
    mvIMUMeas.clear();
    mdT = 0;
    std::cout << "[Preintegrator::Preintegrator] Done" << std::endl;
}
    

void Preintegrator::Integrate(const IMU& imu){
    // std::cout << "[Preintegrator::Integrate] Start" << std::endl;
    mvIMUMeas.emplace_back(imu);
    double dt = imu.mdt;
    mdT += dt;

    Eigen::Vector3d gyr, acc;
    gyr = imu.mGyr - mGyrBias;
    acc = imu.mAcc - mAccBias;
    Eigen::Matrix3d acc_hat = Sophus::SO3d::hat(acc);

    // Eigen::Matrix3d deltaR = LieAlg::Exp(gyr * dt);
    // Eigen::Matrix3d rightJ = RightJacobian(gyr * dt);
    Eigen::Matrix3d deltaR = Sophus::SO3d::exp(gyr * dt).matrix();
    Eigen::Matrix3d rightJ = Sophus::SO3d::leftJacobian(-gyr * dt);

    Eigen::Matrix<double, 9, 9> A;
    A.setIdentity();
    Eigen::Matrix<double, 9, 6> B;
    B.setZero();

    A.block<3, 3>(0, 0) = deltaR.transpose();
    A.block<3, 3>(3, 0) = -mDeltaR * acc_hat * dt;
    A.block<3, 3>(6, 0) = -0.5 * mDeltaR * acc_hat * dt * dt;
    A.block<3, 3>(6, 3) = Eigen::DiagonalMatrix<double, 3>(dt, dt, dt);
    B.block<3, 3>(0, 0) = rightJ * dt;
    B.block<3, 3>(3, 3) = mDeltaR * dt;
    B.block<3, 3>(6, 3) = 0.5 * mDeltaR * dt * dt;
    mCov.block<9, 9>(0, 0) = A * mCov.block<9, 9>(0, 0) * A.transpose() + B * mGyrAccNoise * B.transpose();
    mCov.block<6, 6>(9, 9) += mGyrAccBiasWalk;
    mJacobian = A * mJacobian;
    // std::cout << std::endl << "A: ********************************************************" << std::endl << A << std::endl;
    // std::cout << std::endl << "B: ********************************************************" << std::endl << B << std::endl;
    // std::cout << std::endl << "IMU Cov:***************************************************" << std::endl << mCov.block<9, 9>(0, 0) << std::endl;

    // std::cout << "deltaR: " << std::endl << deltaR << std::endl;
    // std::cout << "before JRg: " << std::endl << mJRbg << std::endl;

    mJPba = mJPba + mJVba * dt - 0.5 * mDeltaR * dt * dt;
    mJPbg = mJPbg + mJVbg * dt - 0.5 * mDeltaR * acc_hat * mJRbg * dt * dt;
    mJVba = mJVba - mDeltaR * dt;
    mJVbg = mJVbg - mDeltaR * acc_hat * mJRbg * dt;
    mJRbg = deltaR.transpose() * mJRbg - rightJ * dt;
    // std::cout << "after JRg: " << std::endl << mJRbg << std::endl;

    IntegrateP(acc, dt);
    IntegrateV(acc, dt);
    mDeltaR *= deltaR;
    mDeltaR = NormalizeRotation(mDeltaR);
    // std::cout << "[Preintegrator::Integrate] Done" << std::endl;
}
/*
void Preintegrator::Integrate1(const IMU& imu0, const IMU& imu1){
    double dt = imu1.mdt - imu0.mdt;
    mdT += dt;

    Eigen::Vector3d _gyr_0, _acc_0, _gyr_1, _acc_1;
    _gyr_0 = imu0.mGyr;
    _acc_0 = imu0.mAcc;
    _gyr_1 = imu1.mGyr;
    _acc_1 = imu1.mAcc;

    Eigen::Matrix3d acc_hat = Sophus::SO3d::hat(acc);

    Eigen::Matrix3d deltaR = Sophus::SO3d::exp(gyr * dt).matrix();
    Eigen::Matrix3d rightJ = Sophus::SO3d::leftJacobian(-gyr * dt);

    Eigen::Matrix<double, 9, 9> A;
    A.setIdentity();
    Eigen::Matrix<double, 9, 6> B;
    B.setZero();

    A.block<3, 3>(0, 0) = deltaR.transpose();
    A.block<3, 3>(3, 0) = -mDeltaR * acc_hat * dt;
    A.block<3, 3>(6, 0) = -0.5 * mDeltaR * acc_hat * dt * dt;
    A.block<3, 3>(6, 3) = Eigen::DiagonalMatrix<double, 3>(dt, dt, dt);
    B.block<3, 3>(0, 0) = rightJ * dt;
    B.block<3, 3>(3, 3) = mDeltaR * dt;
    B.block<3, 3>(6, 3) = 0.5 * mDeltaR * dt * dt;
    mCov.block<9, 9>(0, 0) = A * mCov.block<9, 9>(0, 0) * A.transpose() + B * mGyrAccNoise * B.transpose();
    mCov.block<6, 6>(9, 9) += mGyrAccBiasWalk;
    mJacobian = A * mJacobian;
    mJPba = mJPba + mJVba * dt - 0.5 * mDeltaR * dt * dt;
    mJPbg = mJPbg + mJVbg * dt - 0.5 * mDeltaR * acc_hat * mJRbg * dt * dt;
    mJVba = mJVba - mDeltaR * dt;
    mJVbg = mJVbg - mDeltaR * acc_hat * mJRbg * dt;
    mJRbg = deltaR.transpose() * mJRbg - rightJ * dt;

    IntegrateP(acc, dt);
    IntegrateV(acc, dt);
    mDeltaR *= deltaR;
    mDeltaR = NormalizeRotation(mDeltaR);
}*/

void Preintegrator::IntegrateR(const Eigen::Vector3d& gyr, double dt){
    // Eigen::Matrix3d deltaR = Sophus::SO3d::exp(gyr * dt).matrix();
    Eigen::Matrix3d deltaR = LieAlg::Exp(gyr * dt);
    mDeltaR *= deltaR;
    mDeltaR = NormalizeRotation(mDeltaR);
}

void Preintegrator::IntegrateV(const Eigen::Vector3d& acc, double dt){
    mDeltaV += mDeltaR * acc * dt;
}

void Preintegrator::IntegrateP(const Eigen::Vector3d& acc, double dt){
    mDeltaP += mDeltaV * dt + 0.5 * mDeltaR * acc * dt * dt;
}

void Preintegrator::ReIntegrate(const Eigen::Vector3d& gyrBias, const Eigen::Vector3d& accBias){
    // std::cout << "[Preintegrator::ReIntegrate] Start" << std::endl;
    mDeltaR.setIdentity();
    mDeltaV.setZero();
    mDeltaP.setZero();
    mCov.setZero();
    mJRbg.setZero();
    mJVba.setZero();
    mJVbg.setZero();
    mJPba.setZero();
    mJPbg.setZero();
    mDeltaGyrBias = gyrBias - mGyrBias;
    mDeltaAccBias = accBias - mAccBias;
    mGyrBias = gyrBias;
    mAccBias = accBias;

    auto imus = mvIMUMeas;
    mvIMUMeas.clear();
    mdT = 0;
    for(const auto& imu : imus){
        Integrate(imu);
    }
    // std::cout << "[Preintegrator::ReIntegrate] Done" << std::endl;
}

void Preintegrator::UpdateDeltaPVR(const Eigen::Vector3d& gyrBias, const Eigen::Vector3d& accBias,
                        Eigen::Vector3d& updatedDeltaP, Eigen::Vector3d& updatedDeltaV,
                        Eigen::Matrix3d& updatedDeltaR){
    Eigen::Vector3d deltaGyrBias = gyrBias - mGyrBias;
    Eigen::Vector3d deltaAccBias = accBias - mAccBias;
    updatedDeltaP = UpdateDeltaP(deltaGyrBias, deltaAccBias);
    updatedDeltaV = UpdateDeltaV(deltaGyrBias, deltaAccBias);
    updatedDeltaR = UpdateDeltaR(deltaGyrBias);
}

Eigen::Matrix3d Preintegrator::UpdateDeltaR(const Eigen::Vector3d& deltaGyrBias){
    return NormalizeRotation(mDeltaR * Sophus::SO3d::exp(mJRbg * deltaGyrBias).matrix());
    // return NormalizeRotation(mDeltaR * LieAlg::Exp(mJRbg * deltaGyrBias));
}

Eigen::Vector3d Preintegrator::UpdateDeltaV(const Eigen::Vector3d& deltaGyrBias, const Eigen::Vector3d& deltaAccBias){
    return mDeltaV + mJVbg * deltaGyrBias + mJVba * deltaAccBias;
}

Eigen::Vector3d Preintegrator::UpdateDeltaP(const Eigen::Vector3d& deltaGyrBias, const Eigen::Vector3d& deltaAccBias){
    return mDeltaP + mJPbg * deltaGyrBias + mJPba * deltaAccBias;
}

Eigen::Matrix3d Preintegrator::GetDeltaR() const{
    return mDeltaR;
}

Eigen::Vector3d Preintegrator::GetDeltaV() const{
    return mDeltaV;
}

Eigen::Vector3d Preintegrator::GetDeltaP() const{
    return mDeltaP;
}

Eigen::Matrix3d Preintegrator::GetJRbg() const{
    return mJRbg;
}
Eigen::Matrix3d Preintegrator::GetJVba() const{
    return mJVba;
}
Eigen::Matrix3d Preintegrator::GetJVbg() const{
    return mJVbg;
}
Eigen::Matrix3d Preintegrator::GetJPba() const{
    return mJPba;
}
Eigen::Matrix3d Preintegrator::GetJPbg() const{
    return mJPbg;
}

Eigen::Matrix<double, 15, 15> Preintegrator::GetCov() const{
    return mCov;
}

double Preintegrator::GetDeltaT() const{
    return mdT;
}

Eigen::Vector3d Preintegrator::GetGyrBias() const{
    return mGyrBias;
}

Eigen::Vector3d Preintegrator::GetAccBias() const{
    return mAccBias;
}

std::vector<IMU> Preintegrator::GetIMUs() const{
    return mvIMUMeas;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix3d NormalizeRotation(const Eigen::Matrix3d& R){
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

Eigen::Matrix3d RightJacobian(const Eigen::Vector3d& v){
    Eigen::Matrix3d R;
    double n2 = v.dot(v);
    double n = sqrt(n2);
    // Eigen::Matrix3d v_hat = LieAlg::hat(v);
    Eigen::Matrix3d v_hat = Sophus::SO3d::hat(v);
    if(n < 1e-5){
        R = Eigen::Matrix3d::Identity();
    }
    else{
        R = Eigen::Matrix3d::Identity() - (1 - cos(n)) * v_hat / n2 + (n - sin(n)) * v_hat * v_hat / (n2 * n);
    }
    return R;
}

Eigen::Matrix3d InverseRightJacobian(const Eigen::Vector3d& v){
    Eigen::Matrix3d R;
    double n2 = v.dot(v);
    double n = sqrt(n2);
    // Eigen::Matrix3d v_hat = LieAlg::hat(v);
    Eigen::Matrix3d v_hat = Sophus::SO3d::hat(v);
    if(n < 1e-5){
        R = Eigen::Matrix3d::Identity();
    }
    else{
        R = Eigen::Matrix3d::Identity() + 0.5 * v_hat + (1 / n2 - (1 + cos(n)) / (2 * n * sin(n))) * v_hat * v_hat;
    }
    return R;
}


} // namespace Naive_SLAM_ROS
