//
// Created by hanfuyong on 2022/10/12.
//

#ifndef NAIVE_SLAM_ROS_FEATURETRACKER_H
#define NAIVE_SLAM_ROS_FEATURETRACKER_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include "ImageData.h"
#include "ORBextractor.h"

namespace Naive_SLAM_ROS {

class FeatureTracker {
public:
    FeatureTracker(const std::string &strParamFile);
    void Track(const cv::Mat& img, std::vector<long unsigned int>& vChainIds,
               std::vector<cv::Point2f>& vPtsUn, std::vector<cv::Point2f>& vPts,
               std::vector<int>& vChainLens);

    cv::Mat DrawMatches(const std::string& winName="");

private:
    void FindMatches();


private:
    bool bFirst;

    long unsigned int mnChainId;

    cv::Mat mK;
    cv::Mat mDistCoef;

    int mImgWidth;
    int mImgHeight;
    int mCellSize;
    int mGridCols;
    int mGridRows;

    ImageData mCurrImageData;
    ImageData mPrevImageData;

    std::shared_ptr<ORBextractor> mpORBextractor;
};
}

#endif //NAIVE_SLAM_ROS_FEATURETRACKER_H