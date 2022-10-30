//
// Created by hanfuyong on 2022/10/27
//

#ifndef NAIVESLAMROS_INITIALIZER_H
#define NAIVESLAMROS_INITIALIZER_H


#include "FeatureManager.h"
#include "Frame.h"
#include "TypeConverter.h"
#include <opencv2/core/eigen.hpp>

namespace Naive_SLAM_ROS{

class Initializer{
public:
    Initializer(int matchNumTh, float parallaxTh, const Eigen::Matrix3d& K, FeatureManager* pFM);

    bool VisualInitS1(std::vector<Frame>& qFrames,std::vector<Eigen::Vector3d>& vPts3D, 
        std::vector<Eigen::Vector2d>& vPts2D1, std::vector<Eigen::Vector2d>& vPts2D2,
        std::vector<unsigned long>& vChainIds);

    bool VisualInitS2(std::vector<Frame>& qFrames);

    std::vector<Eigen::Vector3d> TriangulateTwoFrame(const Eigen::Matrix3d& Rcw1, const Eigen::Vector3d& tcw1,
        const Eigen::Matrix3d& Rcw2, const Eigen::Vector3d& tcw2,
        const std::vector<Eigen::Vector2d>& vPts2D1, const std::vector<Eigen::Vector2d>& vPts2D2,
        const std::vector<unsigned long>& vChainIds);

private:
    int mMatchNumTh;
    float mParallaxTh;
    Eigen::Matrix3d mK;
    int mInitIdx;
    FeatureManager* mpFM;
};
    
} // namespace Naive_SLAM_ROS


#endif