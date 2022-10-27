//
// Created by hanfuyong on 2022/10/27
//

#ifndef NAIVESLAMROS_INITIALIZER_H
#define NAIVESLAMROS_INITIALIZER_H

#include "FeatureManager.h"
#include "Frame.h"

namespace Naive_SLAM_ROS{

class Initializer{
public:
    Initializer(int matchNumTh, float parallaxTh, const cv::Mat& K);
    bool VisualInitS1(std::vector<Frame>& qFrames, FeatureManager* pFM,
        std::vector<cv::Vec3f>& vPts3D, std::vector<cv::Vec2f>& vPts2D1, std::vector<cv::Vec2f>& vPts2D2,
        std::vector<unsigned long>& vChainIds);
    bool VisualInitS2(std::vector<Frame>& qFrames, FeatureManager* pFM);

private:
    int mMatchNumTh;
    float mParallaxTh;
    cv::Mat mK;
    int mInitIdx;
};
    
} // namespace Naive_SLAM_ROS


#endif