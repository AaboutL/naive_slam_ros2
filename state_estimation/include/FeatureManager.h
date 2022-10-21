//
// Created by hanfuyong on 2022/10/21
//

#ifndef NAIVESLAMROS_FEATUREMANAGER_H
#define NAIVESLAMROS_FEATUREMANAGER_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include "Frame.h"

namespace Naive_SLAM_ROS{

class Feature{
public:
    Feature(long unsigned int chainId, long unsigned int frameId, const cv::Vec3f& ptUn, const cv::Vec2f& ptUnOffset);
    const long unsigned int mChainId;
    const long unsigned int mFrameId;
    cv::Vec3f mPtUn;
    cv::Vec2f mPtUnOffset;
};

class FeatureChain{
public:
    FeatureChain(long unsigned int chainId);
    FeatureChain(const FeatureChain& featureChain);

    void AddFeature(const Feature& feat);
    int GetChainLen() const;
    cv::Vec2f GetOffset() const;
    void EraseFront(long unsigned int frameId);
    void EraseBack(long unsigned int frameId);

public:
    const long unsigned int mChainId;
    std::vector<Feature> mvFeatures;
};

class FeatureManager{
public:
    FeatureManager();
    void Manage(const Frame& frame, long unsigned int frameId);
    int GetChains(int chainLen, std::vector<FeatureChain>& vChains) const;
    void EraseFront(long unsigned int frameId);
    void EraseBack(long unsigned int frameId);

private:
    std::unordered_map<long unsigned int, FeatureChain> mmChains;
};

    
} // namespace Naive_SLAM_ROS


#endif