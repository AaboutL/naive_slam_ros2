//
// Created by hanfuyong on 2022/10/27
//

#ifndef NAIVESLAMROS_INITIALIZER_H
#define NAIVESLAMROS_INITIALIZER_H

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__ // same but can not remove
#endif

#include <eigen3/Eigen/Dense>
#include <sophus/se3.hpp>
#include <opencv2/core/eigen.hpp>

#include "FeatureManager.h"
#include "Frame.h"
#include "TypeConverter.h"
#include "Optimizer.h"
#include "LieAlgebra.h"

namespace Naive_SLAM_ROS{

class Initializer{
public:
    Initializer(int matchNumTh, float parallaxTh, const Eigen::Matrix3d& K, FeatureManager* pFM,
                Sophus::SE3d& Tbc);

    void Reset();

    bool VisualOnlyInitS1(std::vector<Frame*>& qFrames);

    bool VisualOnlyInitS2(std::vector<Frame*>& qFrames);

    void NormalizePoseAndPoint(Frame* pF1, Frame* pF2, std::vector<Eigen::Vector3d>& vPts3D, 
        const std::vector<unsigned long>& vChainIds);

    bool VisualInertialInit(std::vector<Frame*>& vpFrames);
    void VisualInertialAlign(std::vector<Frame*>& vpFrames, const Eigen::Matrix3d& Rwg,
                             const Eigen::Vector3d& gyrBias, const Eigen::Vector3d& accBias, double scale);

    void SolveGyrBias(std::vector<Frame*>& vpFrames);
    bool LinearAlignment(std::vector<Frame*>& vpFrames, Eigen::Vector3d& g, Eigen::VectorXd& x);
    Eigen::MatrixXd TangentBasis(const Eigen::Vector3d &g0);
    void RefineGravity(std::vector<Frame*>& vpFrames, Eigen::Vector3d& g, Eigen::VectorXd& x);
    bool VIAlign(std::vector<Frame*>& vpFrames);

private:
    int mMatchNumTh;
    float mParallaxTh;
    Eigen::Matrix3d mK;
    int mInitIdx;
    FeatureManager* mpFM;

    Sophus::SE3d mTbc;
    Sophus::SE3d mTcb;

    Eigen::Vector3d mG;

};
    
} // namespace Naive_SLAM_ROS


#endif