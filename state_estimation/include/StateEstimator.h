//
// Created by hanfuyong on 2022/10/20
//

#ifndef NAIVESLAMROS_SLIDINGWINDOW_H
#define NAIVESLAMROS_SLIDINGWINDOW_H

#include <unordered_map>

#include "Frame.h"
#include "IMU.h"
#include "FeatureManager.h"

namespace Naive_SLAM_ROS {

class StateEstimator{
public:
    StateEstimator(int windowSize);

    enum State{
        INIT = 1,
        ESTIMATE = 2,
        LOST = 3
    };

    void Estimate(const std::pair<Frame, std::vector<IMU>>& pMeas);

    bool VisualInit();

private:
    const int mWindowSize;
    long unsigned int mFrameId;
    State mState;
    // std::unordered_map<long unsigned int, FeatureChain> mmChains;
    FeatureManager* mpFM;
    std::deque<Frame> mqFrames;
};

    
} // namespace Naive_SLAM_ROS

#endif