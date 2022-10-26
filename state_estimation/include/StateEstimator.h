//
// Created by hanfuyong on 2022/10/20
//

#ifndef NAIVESLAMROS_SLIDINGWINDOW_H
#define NAIVESLAMROS_SLIDINGWINDOW_H

#include <unordered_map>

#include "Frame.h"
#include "IMU.h"
#include "FeatureManager.h"
#include "GeometryFunc.h"
#include "Optimizer.h"

namespace Naive_SLAM_ROS {

class StateEstimator{
public:
    StateEstimator(const std::string& strParamFile);
    StateEstimator(int windowSize, const cv::Mat& K);

    enum State{
        INIT = 1,
        ESTIMATE = 2,
        LOST = 3
    };

    void Estimate(const std::pair<Frame, std::vector<IMU>>& pMeas);

    bool VisualInit();
    int SolveRelativePose(const std::vector<cv::Vec2f> &vPts0, const std::vector<cv::Vec2f> &vPts1,
                          cv::Mat &R10, cv::Mat &t10);

private:
    int mWindowSize;
    unsigned long mFrameId;
    State mState;
    FeatureManager* mpFM;
    std::deque<Frame> mqFrames;

    cv::Mat mK;
};

    
} // namespace Naive_SLAM_ROS

#endif