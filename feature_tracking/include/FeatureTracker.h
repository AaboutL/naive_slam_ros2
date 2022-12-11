//
// Created by hanfuyong on 2022/10/12.
//

#ifndef NAIVESLAMROS_FEATURETRACKER_H
#define NAIVESLAMROS_FEATURETRACKER_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include "ImageData.h"
#include "ORBextractor.h"

namespace Naive_SLAM_ROS {

class FeatureTracker {
public:
    FeatureTracker(const std::string &strParamFile);
    void Track(const cv::Mat& img, std::vector<unsigned long>& vChainIds,
               std::vector<cv::Point2f>& vPtsUn, std::vector<cv::Point2f>& vPts,
               std::vector<cv::Point2f>& vPtUnOffsets,
               std::vector<int>& vChainLens);

    void TrackHarris(const cv::Mat& img, std::vector<unsigned long>& vChainIds,
               std::vector<cv::Point2f>& vPtsUn, std::vector<cv::Point2f>& vPts,
               std::vector<cv::Point2f>& vPtUnOffsets,
               std::vector<int>& vChainLens);

    cv::Mat DrawMatches(const std::string& winName="");
    cv::Mat DrawTrack(const std::string& winName="");

private:
    void FindMatches();
    void FindMatchesHarris();

    void RejectByFundamental();
    void RejectByFundamentalHarris();
    void ReduceVector(std::vector<cv::Point2f>& v, std::vector<uchar>& status);
    void ReduceVector(std::vector<int>& v, std::vector<uchar>& status);
    void ReduceVector(std::vector<size_t>& v, std::vector<uchar>& status);

private:
    bool bFirst;

    unsigned long mnChainId;

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

    int mnFeatureNum;
};
}

#endif //NAIVE_SLAM_ROS_FEATURETRACKER_H
