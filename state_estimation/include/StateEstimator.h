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
#include "TypeConverter.h"

namespace Naive_SLAM_ROS {

class StateEstimator{
public:
    StateEstimator(const std::string& strParamFile);

    enum State{
        INIT = 1,
        ESTIMATE = 2,
        LOST = 3
    };

    void Estimate(const std::pair<PointCloud, std::vector<IMU>>& pMeas);

    bool Initialize();
    int SolveRelativePose(const std::vector<Eigen::Vector2d> &vPts0, const std::vector<Eigen::Vector2d> &vPts1,
                          cv::Mat &R10, cv::Mat &t10);

    void Preintegrate(Frame* frame, const std::vector<IMU>& vIMUs);
    void Marginalize(int margPos=0);
    void Reset();

private:
    int mWindowSize;
    unsigned long mFrameId;
    State mState;
    FeatureManager* mpFM;
    std::vector<Frame*> mvpFrames;

    Eigen::Matrix3d mK;

    // double mdLastTimestamp; // last frame timestamp

    // Preintegration data
    Eigen::Vector3d mLastGyrBias;
    Eigen::Vector3d mLastAccBias;
    double mGyrNoise;
    double mAccNoise;
    double mGyrBiasWalk;
    double mAccBiasWalk;
    double mIMUFrequency;

    // Eigen::Matrix4d mTbc;
    Sophus::SE3d mTbc;

    Initializer* mpInitializer;
    Frame* mpLastFrame;
    std::vector<IMU> mvLastIMUs;
};

    
} // namespace Naive_SLAM_ROS

#endif