//
// Created by hanfuyong on 2022/10/12.
//

#include "FeatureTracker.h"

namespace Naive_SLAM_ROS {

FeatureTracker::FeatureTracker(const std::string &strParamFile):
    bFirst(true), mnChainId(0){
    cv::FileStorage fs(strParamFile.c_str(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "[Estimator] Param file not exist..." << std::endl;
        exit(0);
    }

    mK = cv::Mat::eye(3, 3, CV_32FC1);
    mK.at<float>(0, 0) = fs["Camera.fx"];
    mK.at<float>(1, 1) = fs["Camera.fy"];
    mK.at<float>(0, 2) = fs["Camera.cx"];
    mK.at<float>(1, 2) = fs["Camera.cy"];

    mDistCoef = cv::Mat::zeros(4, 1, CV_32FC1);
    mDistCoef.at<float>(0, 0) = fs["Camera.k1"];
    mDistCoef.at<float>(1, 0) = fs["Camera.k2"];
    mDistCoef.at<float>(2, 0) = fs["Camera.p1"];
    mDistCoef.at<float>(3, 0) = fs["Camera.p2"];

    mImgWidth = fs["Camera.width"];
    mImgHeight = fs["Camera.height"];
    mCellSize = fs["cell_size"];
    mGridCols = (int) std::ceil((float) mImgWidth / (float) mCellSize);
    mGridRows = (int) std::ceil((float) mImgHeight / (float) mCellSize);
    mpORBextractor = std::make_shared<ORBextractor>(fs["feature_num"], 1, 1, 20, 7);
}

void FeatureTracker::Track(const cv::Mat& img, 
                           std::vector<long unsigned int>& vChainIds,
                           std::vector<cv::Point2f>& vPtsUn,
                           std::vector<cv::Point2f>& vPts){
    mCurrImageData = ImageData(img, mK, mDistCoef, mImgWidth, mImgHeight,
                               mCellSize, mGridCols, mGridRows, mpORBextractor);
    if(bFirst){
        mPrevImageData = ImageData(mCurrImageData);
        bFirst = false;
    }
    else{
        FindMatches();
    }
    PackingResult(vChainIds, vPtsUn, vPts);
}

void FeatureTracker::FindMatches(){
    std::vector<uchar> vStatus;
    std::vector<float> vErr;
    std::vector<cv::Point2f> vPtsLK;
    cv::calcOpticalFlowPyrLK(mPrevImageData.mImg, mCurrImageData.mImg, mPrevImageData.mvPts,
                             vPtsLK, vStatus, vErr, cv::Size(21, 21), 3);

    std::vector<int> vMatches12(mPrevImageData.N, -1);
    std::vector<int> vMatches21(mCurrImageData.N, -1);
    std::vector<int> vMatchDist21(mCurrImageData.N, 256);
    for (int i = 0; i < mPrevImageData.N; i++) {
        if (vStatus[i] != 1)
            continue;
        cv::Point2f pt = vPtsLK[i];
        cv::Mat description = mPrevImageData.mDescriptors.row(i);
        float radiusTh = 15.0;
        int curPtId;
        int bestDist = mCurrImageData.SearchGrid(pt, description, radiusTh, curPtId);
        if (bestDist < 50) {
            if (vMatches21[curPtId] == -1) {
                vMatches12[i] = curPtId;
                vMatches21[curPtId] = i;
                vMatchDist21[curPtId] = bestDist;

                int chainLen = mPrevImageData.mvChainLens[i];
                if(chainLen < 2) { // 新chain
                    chainLen++;
                    mPrevImageData.mvChainIds[i] = mnChainId;
                    mCurrImageData.mvChainIds[curPtId] = mnChainId;
                    mPrevImageData.mvChainLens[i] = chainLen;
                    mCurrImageData.mvChainLens[curPtId] = chainLen;
                    mnChainId++;
                }
                else{
                    mCurrImageData.mvChainIds[curPtId] = mPrevImageData.mvChainIds[i];
                    chainLen++;
                    mPrevImageData.mvChainLens[i] = chainLen;
                    mCurrImageData.mvChainLens[curPtId] = chainLen;
                }
            } else {
                if (bestDist < vMatchDist21[curPtId]) {
                    vMatches12[i] = curPtId;
                    vMatches12[vMatches21[curPtId]] = -1;
                    vMatchDist21[curPtId] = bestDist;

                    auto correctChainId = mPrevImageData.mvChainIds[i];
                    mCurrImageData.mvChainIds[curPtId] = correctChainId;

                    mPrevImageData.mvChainLens[vMatches21[curPtId]]--;
                    if(mPrevImageData.mvChainLens[vMatches21[curPtId]] == 1){
                        mPrevImageData.mvChainIds[vMatches21[curPtId]] = -1;
                    }

                    vMatches21[curPtId] = i;
                }
            }
        }
    }
}

void FeatureTracker::PackingResult(std::vector<long unsigned int>& vChainIds,
                                   std::vector<cv::Point2f>& vPtsUn,
                                   std::vector<cv::Point2f>& vPts) {
    vChainIds.reserve(mCurrImageData.N);
    vPtsUn.reserve(mCurrImageData.N);
    vPts.reserve(mCurrImageData.N);
    for(int i = 0; i < mCurrImageData.N; i++){
        if(mCurrImageData.mvChainLens[i] > 1){
            vChainIds.push_back(i);
            vPtsUn.emplace_back(mCurrImageData.mvPtsUn[i]);
            vPts.emplace_back(mCurrImageData.mvPts[i]);
        }
    }
}


} // namespace Naive_SLAM_ROS
