//
// Created by hanfuyong on 2022/10/21
//

#ifndef NAIVESLAMROS_FEATUREMANAGER_H
#define NAIVESLAMROS_FEATUREMANAGER_H

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__ // 与上一行一样，但是不能删除
#endif

#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "Frame.h"

namespace Naive_SLAM_ROS{

class Feature{
public:
    Feature(const Feature& feature);
    Feature(unsigned long chainId, unsigned long frameId, const Eigen::Vector2d& ptUn, const Eigen::Vector2d& ptUnOffset);
    unsigned long mChainId;
    unsigned long mFrameId;
    Eigen::Vector2d mPtUn;
    Eigen::Vector2d mPtUnOffset;
};

class FeatureChain{
public:
    FeatureChain(unsigned long chainId, int windowSize, int startId);
    FeatureChain(const FeatureChain& featureChain);

    void AddFeature(const Feature& feat);
    int GetChainLen() const;
    Eigen::Vector2d GetOffset() const;
    void EraseFront();
    void EraseBack();

public:
    unsigned long mChainId;
    int mWindowSize;
    std::vector<Feature> mvFeatures;
    int mStartIdx;
    Eigen::Vector3d mWorldPos; // position in world frame
    bool mbGood;
    bool mbPosSet;
};

class FeatureManager{
public:
    FeatureManager(int windowSize);
    void Manage(const PointCloud& pc, unsigned long frameId, int startId);
    int GetChains(int chainLen, std::vector<FeatureChain>& vChains) const;
    const std::unordered_map<unsigned long, FeatureChain>& GetChains() const;

    void EraseFront();
    void EraseBack();

    int GetMatches(int pos1, int pos2, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& vMatches) const;
    int GetMatches(int pos1, int pos2, std::vector<Eigen::Vector2d>& vPts1, std::vector<Eigen::Vector2d>& vPts2, 
                   std::vector<unsigned long>& vChainIds) const;
    int GetMatches(int pos1, int pos2, std::vector<Eigen::Vector3d>& vPts3D, std::vector<Eigen::Vector2d>& vPts2D, 
                   std::vector<Eigen::Vector2d>& vPts1, std::vector<Eigen::Vector2d>& vPts2,
                   std::vector<unsigned long>& vChainIds) const;

    void SetWorldPos(unsigned long chainId, const Eigen::Vector3d& pos);
    void SetChainGood(unsigned long chainId, bool bGood);
    void UpdateWorldPos(unsigned long chainId, const Eigen::Vector3d& pos);

    bool IsChainGood(unsigned long chainId);
    bool IsChainPosSet(unsigned long chainId);
    void Reset();

private:
    std::unordered_map<unsigned long, FeatureChain> mmChains;
    int mWindowSize;
};

    
} // namespace Naive_SLAM_ROS


#endif