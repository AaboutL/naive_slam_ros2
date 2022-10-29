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
#include "Initializer.h"

namespace Naive_SLAM_ROS {

class StateEstimator{
public:
    StateEstimator(const std::string& strParamFile);

    enum State{
        INITS1 = 1,
        INITS2 = 2,
        VIINIT = 3,
        ESTIMATE = 4,
        LOST = 5
    };

    void Estimate(const std::pair<Frame, std::vector<IMU>>& pMeas);

    int VisualInit();
    int SolveRelativePose(const std::vector<Eigen::Vector2d> &vPts0, const std::vector<Eigen::Vector2d> &vPts1,
                          cv::Mat &R10, cv::Mat &t10);

private:
    int mWindowSize;
    unsigned long mFrameId;
    State mState;
    std::shared_ptr<FeatureManager> mpFM;
    std::vector<Frame> mvFrames;

    Eigen::Matrix3d mK;
    Initializer* mpInitializer;
};

    
} // namespace Naive_SLAM_ROS

#endif