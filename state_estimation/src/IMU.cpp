//
// Created by hanfuyong on 2022/10/18.
//

#include "IMU.h"

namespace Naive_SLAM_ROS{

IMU::IMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr):
mdTimestamp(timestamp), mAcc(acc), mGyr(gyr), mVel(Eigen::Vector3d::Zero()),
mAccBias(Eigen::Vector3d::Zero()), mGyrBias(Eigen::Vector3d::Zero()){
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// Preintegrator::Preintegrator(const Eigen::Matrix<double, 15, 15>& cov,
//                              const Eigen::Matrix3d& JRbg,
//                              const Eigen::Matrix3d& JVba,
//                              const Eigen::Matrix3d& JVbg,
//                              const Eigen::Matrix3d& JPba,
//                              const Eigen::Matrix3d& JPbg,
//                              const Eigen::DiagonalMatrix<double, 6>& noiseGyrAcc,
//                              const Eigen::DiagonalMatrix<double, 6>& noiseGyrAccWalk):
// mdLastTimestamp(-1), mDeltaR(Eigen::Matrix3d::Identity()), mDeltaV(Eigen::Vector3d::Zero()),
// mDeltaP(Eigen::Vector3d::Zero()), mCov(cov), 
// mJRbg(JRbg),mJVba(JVba),mJVbg(JVbg),mJPba(JPba),mJPbg(JPbg),
// mNoiseGyrAcc(noiseGyrAcc), mNoiseGyrAccWalk(noiseGyrAccWalk){
// }

Preintegrator::Preintegrator(const Eigen::DiagonalMatrix<double, 6>& noiseGyrAcc,
                             const Eigen::DiagonalMatrix<double, 6>& noiseGyrAccWalk,
                             const Eigen::Vector3d& gyrBias, const Eigen::Vector3d& accBias):
mdLastTimestamp(-1), mNoiseGyrAcc(noiseGyrAcc), mNoiseGyrAccWalk(noiseGyrAccWalk),
mGyrBias(gyrBias), mAccBias(accBias){
    mDeltaR.setIdentity();
    mDeltaV.setZero();
    mDeltaP.setZero();
    mCov.setZero();
    mJRbg.setZero();
    mJVba.setZero();
    mJVbg.setZero();
    mJPba.setZero();
    mJPbg.setZero();
    mDeltaGyrBias.setZero();
    mDeltaAccBias.setZero();
    mvIMUMeas.clear();
}
    
void Preintegrator::Integrate(const IMU& imu){
    mvIMUMeas.emplace_back(imu);
    double dt = imu.mdTimestamp - mdLastTimestamp;
    Eigen::Vector3d gyr, acc;
    gyr = imu.mGyr - mGyrBias;
    acc = imu.mAcc - mAccBias;
    Eigen::Matrix3d acc_hat = LieAlg::hat(acc);

    Eigen::Matrix3d deltaR = LieAlg::Exp(gyr * dt);
    Eigen::Matrix3d rightJ = RightJacobian(gyr);

    Eigen::Matrix<double, 9, 9> A;
    Eigen::Matrix<double, 9, 6> B;

    A.block<3, 3>(0, 0) = deltaR.transpose();
    A.block<3, 3>(3, 0) = -mDeltaR * acc_hat * dt;
    A.block<3, 3>(6, 0) = -0.5 * mDeltaR * acc_hat * dt * dt;
    A.block<3, 3>(6, 3) *= dt;
    B.block<3, 3>(0, 0) = rightJ * dt;
    B.block<3, 3>(3, 3) = mDeltaR * dt;
    B.block<3, 3>(6, 3) = 0.5 * mDeltaR * dt * dt;
    mCov.block<9, 9>(0, 0) = A * mCov.block<9, 9>(0, 0) * A.transpose() + B * mNoiseGyrAcc * B.transpose();
    mCov.block<6, 6>(9, 9) += mNoiseGyrAccWalk;

    mJPba = mJPba + mJVba * dt - 0.5 * mDeltaR * dt * dt;
    mJPbg = mJPbg + mJVbg * dt - 0.5 * mDeltaR * acc_hat * mJRbg * dt * dt;
    mJVba = mJVba - mDeltaR * dt;
    mJVbg = mJVbg - mDeltaR * acc_hat * mJRbg * dt;
    mJRbg = deltaR.transpose() * mJRbg - rightJ * dt;

    IntegrateP(acc, dt);
    IntegrateV(acc, dt);
    mDeltaR *= deltaR;
    mDeltaR = NormalizeRotation(mDeltaR);
}

void Preintegrator::IntegrateR(const Eigen::Vector3d& gyr, double dt){
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
    for(const auto& imu : mvIMUMeas){
        Integrate(imu);
    }
}

void Preintegrator::UpdateDeltaPVR(const Eigen::Vector3d& gyrBias, const Eigen::Vector3d& accBias,
                        Eigen::Vector3d& updateDeltaP, Eigen::Vector3d& updateDeltaV,
                        Eigen::Matrix3d& updateDeltaR){
    Eigen::Vector3d deltaGyrBias = gyrBias - mGyrBias;
    Eigen::Vector3d deltaAccBias = accBias - mAccBias;
    updateDeltaP = UpdateDeltaP(deltaGyrBias, deltaAccBias);
    updateDeltaV = UpdateDeltaV(deltaGyrBias, deltaAccBias);
    updateDeltaR = UpdateDeltaR(deltaGyrBias);
}

Eigen::Matrix3d Preintegrator::UpdateDeltaR(const Eigen::Vector3d& deltaGyrBias){
    return NormalizeRotation(mDeltaR * LieAlg::Exp(mJRbg * deltaGyrBias));
}

Eigen::Vector3d Preintegrator::UpdateDeltaV(const Eigen::Vector3d& deltaGyrBias, const Eigen::Vector3d& deltaAccBias){
    return mDeltaV + mJVbg * deltaGyrBias + mJVba * deltaAccBias;
}

Eigen::Vector3d Preintegrator::UpdateDeltaP(const Eigen::Vector3d& deltaGyrBias, const Eigen::Vector3d& deltaAccBias){
    return mDeltaP + mJPbg * deltaGyrBias + mJPba * deltaAccBias;
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
    Eigen::Matrix3d v_hat = LieAlg::hat(v);
    if(n < 1e-4){
        R = Eigen::Matrix3d::Identity();
    }
    else{
        R = Eigen::Matrix3d::Identity() + (1 - cos(n)) * v_hat / n2 + (n - sin(n)) * v_hat * v_hat / (n2 * n);
    }
    return R;
}

Eigen::Matrix3d InverseRightJacobian(const Eigen::Vector3d& v){
    Eigen::Matrix3d R;
    double n2 = v.dot(v);
    double n = sqrt(n2);
    Eigen::Matrix3d v_hat = LieAlg::hat(v);
    if(n < 1e-4){
        R = Eigen::Matrix3d::Identity();
    }
    else{
        R = Eigen::Matrix3d::Identity() + 0.5 * v_hat + (1 / n2 - (1 + cos(n)) / (2 * n * sin(n))) * v_hat * v_hat;
    }
    return R;
}

} // namespace Naive_SLAM_ROS
