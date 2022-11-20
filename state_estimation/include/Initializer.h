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

    std::vector<Eigen::Vector3d> TriangulateTwoFrame(const Eigen::Matrix3d& Rcw1, const Eigen::Vector3d& tcw1,
        const Eigen::Matrix3d& Rcw2, const Eigen::Vector3d& tcw2,
        const std::vector<Eigen::Vector2d>& vPts2D1, const std::vector<Eigen::Vector2d>& vPts2D2,
        const std::vector<unsigned long>& vChainIds);

    void NormalizePoseAndPoint(Frame* pF1, Frame* pF2, std::vector<Eigen::Vector3d>& vPts3D, 
        const std::vector<unsigned long>& vChainIds);

    bool VisualInertialInit(std::vector<Frame*>& vpFrames);
    void VisualInertialAlign(std::vector<Frame*>& vpFrames, const Eigen::Matrix3d& Rwg,
                             const Eigen::Vector3d& gyrBias, const Eigen::Vector3d& accBias, double scale);

private:
    int mMatchNumTh;
    float mParallaxTh;
    Eigen::Matrix3d mK;
    int mInitIdx;
    FeatureManager* mpFM;

    Sophus::SE3d mTbc;
    Sophus::SE3d mTcb;

};
    
} // namespace Naive_SLAM_ROS


#endif