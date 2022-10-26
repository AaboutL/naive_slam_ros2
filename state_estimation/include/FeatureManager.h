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
    Feature(const Feature& feature);
    Feature(unsigned long chainId, unsigned long frameId, const cv::Vec3f& ptUn, const cv::Vec2f& ptUnOffset);
    unsigned long mChainId;
    unsigned long mFrameId;
    cv::Vec3f mPtUn;
    cv::Vec2f mPtUnOffset;
};

class FeatureChain{
public:
    FeatureChain(unsigned long chainId, int windowSize, int startId);
    FeatureChain(const FeatureChain& featureChain);

    void AddFeature(const Feature& feat);
    int GetChainLen() const;
    cv::Vec2f GetOffset() const;
    void EraseFront();
    void EraseBack();

public:
    unsigned long mChainId;
    int mWindowSize;
    std::vector<Feature> mvFeatures;
    int mStartIdx;
    cv::Vec3f mWorldPos; // position in world frame
};

class FeatureManager{
public:
    FeatureManager(int windowSize);
    void Manage(const Frame& frame, unsigned long frameId, int startId);
    int GetChains(int chainLen, std::vector<FeatureChain>& vChains) const;
    void EraseFront();
    void EraseBack();
    int GetMatches(int pos1, int pos2, std::vector<std::pair<cv::Vec2f, cv::Vec2f>>& vMatches) const;
    // int GetMatches(int pos1, int pos2, std::vector<cv::Vec2f>& vPts1, std::vector<cv::Vec2f>& vPts2) const;
    int GetMatches(int pos1, int pos2, std::vector<cv::Vec2f>& vPts1, std::vector<cv::Vec2f>& vPts2, 
                   std::vector<unsigned long>& vChainIds) const;
    void SetPosition(unsigned long chainId, const cv::Vec3f& pos);

private:
    std::unordered_map<unsigned long, FeatureChain> mmChains;
    int mWindowSize;
};

    
} // namespace Naive_SLAM_ROS


#endif