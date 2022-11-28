//
// Created by hanfuyong on 2022/10/18
//

#ifndef IMU_H
#define IMU_H

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__ // same but can not remove
#endif

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "LieAlgebra.h"
#include <sophus/se3.hpp>

namespace Naive_SLAM_ROS
{

class IMU{
public:
    IMU(){}
    IMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr, double dt);
    IMU(const IMU& imu);
    IMU(double timestamp, const std::string& sTimestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr, double dt);

public:
    double mdTimestamp;
    Eigen::Vector3d mAcc;
    Eigen::Vector3d mGyr;
    Eigen::Vector3d mVel;
    double mdt; // duration between last and cur
    std::string msTimestamp;
};

class Preintegrator{
public:
    Preintegrator(){}

    Preintegrator(const Eigen::DiagonalMatrix<double, 6>& noiseGyrAcc,
                  const Eigen::DiagonalMatrix<double, 6>& noiseGyrAccWalk,
                  const Eigen::Vector3d& gyrBias, const Eigen::Vector3d& accBias);

    Preintegrator(double gyrNoise, double accNoise, double gyrBiasWalk, double accBiasWalk,
                  const Eigen::Vector3d& gyrBias, const Eigen::Vector3d& accBias);

    void Integrate(const IMU& imu);
    void ReIntegrate(const Eigen::Vector3d& gyrBias, const Eigen::Vector3d& accBias);
    void UpdateDeltaPVR(const Eigen::Vector3d& gyrBias, const Eigen::Vector3d& accBias,
                        Eigen::Vector3d& updatedDeltaP, Eigen::Vector3d& updatedDeltaV,
                        Eigen::Matrix3d& updatedDeltaR);
    
    Eigen::Matrix3d GetDeltaR() const;
    Eigen::Vector3d GetDeltaV() const;
    Eigen::Vector3d GetDeltaP() const;

    Eigen::Matrix3d GetJRbg() const;
    Eigen::Matrix3d GetJVba() const;
    Eigen::Matrix3d GetJVbg() const;
    Eigen::Matrix3d GetJPba() const;
    Eigen::Matrix3d GetJPbg() const;
    Eigen::Matrix<double, 15, 15> GetCov() const;
    double GetDeltaT() const;
    Eigen::Vector3d GetGyrBias() const;
    Eigen::Vector3d GetAccBias() const;
    std::vector<IMU> GetIMUs() const;

private:
    void IntegrateR(const Eigen::Vector3d& gyr, double dt);
    void IntegrateV(const Eigen::Vector3d& acc, double dt);
    void IntegrateP(const Eigen::Vector3d& acc, double dt);
    Eigen::Matrix3d UpdateDeltaR(const Eigen::Vector3d& deltaGyrBias);
    Eigen::Vector3d UpdateDeltaV(const Eigen::Vector3d& deltaGyrBias, const Eigen::Vector3d& deltaAccBias);
    Eigen::Vector3d UpdateDeltaP(const Eigen::Vector3d& deltaGyrBias, const Eigen::Vector3d& deltaAccBias);

private:
    double mdLastTimestamp;
    Eigen::Matrix3d mDeltaR;
    Eigen::Vector3d mDeltaV;
    Eigen::Vector3d mDeltaP;
    Eigen::Matrix<double, 15, 15> mCov;

    // jacobian for bias update
    Eigen::Matrix3d mJRbg;
    Eigen::Matrix3d mJVba;
    Eigen::Matrix3d mJVbg;
    Eigen::Matrix3d mJPba;
    Eigen::Matrix3d mJPbg;

    Eigen::DiagonalMatrix<double, 6> mGyrAccNoise;
    Eigen::DiagonalMatrix<double, 6> mGyrAccBiasWalk;

    Eigen::Vector3d mGyrBias;
    Eigen::Vector3d mAccBias;

    Eigen::Vector3d mDeltaGyrBias; // oldBias + mDeltaBias = updatedBias
    Eigen::Vector3d mDeltaAccBias;

    std::vector<IMU> mvIMUMeas;
    double mdT;
};

Eigen::Matrix3d NormalizeRotation(const Eigen::Matrix3d& R);
Eigen::Matrix3d RightJacobian(const Eigen::Vector3d& v);
Eigen::Matrix3d InverseRightJacobian(const Eigen::Vector3d& v);
    
} // namespace Naive_SLAM_ROS


#endif