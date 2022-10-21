//
// Created by hanfuyong on 2022/10/12.
//

#include "FeatureTracker.h"
#include <unordered_map>

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
                           std::vector<cv::Point2f>& vPts,
                           std::vector<cv::Point2f>& vPtUnOffsets,
                           std::vector<int>& vChainLens){
    mCurrImageData = ImageData(img, mK, mDistCoef, mImgWidth, mImgHeight,
                               mCellSize, mGridRows, mGridCols, mpORBextractor);
    if(bFirst){
        mPrevImageData = ImageData(mCurrImageData);
        bFirst = false;
    }
    else{
        FindMatches();
        // DrawMatches("Matches");
        RejectByFundamental();
        mPrevImageData = ImageData(mCurrImageData);
    }

    vChainIds = mCurrImageData.mvChainIds;
    vPtsUn = mCurrImageData.mvPtsUn;
    vPts = mCurrImageData.mvPts;
    vChainLens = mCurrImageData.mvChainLens;
    vPtUnOffsets = mCurrImageData.mvPtUnOffsets;
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
        int bestDist = mCurrImageData.SearchGrid(pt, description, radiusTh*6, curPtId);
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
            } 
            else {
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

void FeatureTracker::RejectByFundamental(){
    std::unordered_map<int, std::pair<cv::Point2f, cv::Point2f>> matchedPtsUn;
    std::unordered_map<int, int> mChainIdWithPtId;
    for(int i = 0; i < mCurrImageData.N; i++){
        if(mCurrImageData.mvChainLens[i] >= 2){
            matchedPtsUn[mCurrImageData.mvChainIds[i]] = std::make_pair(cv::Point2f(), mCurrImageData.mvPtsUn[i]);
            mChainIdWithPtId[mCurrImageData.mvChainIds[i]] = i;
        }
    }

    for(int i = 0; i < mPrevImageData.N; i++){
        if(matchedPtsUn.find(mPrevImageData.mvChainIds[i]) != matchedPtsUn.end()){
            matchedPtsUn[mPrevImageData.mvChainIds[i]].first = mPrevImageData.mvPtsUn[i];
            mCurrImageData.mvPtUnOffsets[mChainIdWithPtId[mPrevImageData.mvChainIds[i]]] = 
                mCurrImage.Data.mvPtsUn[mChainIdWithPtId[mPrevImageData.mvChainIds[i]]] - mPrevImageData.mvPtsUn[i];
        }
    }

    std::vector<cv::Point2f> vPrevPtsUn, vCurrPtsUn, vPG, vCG;
    std::unordered_map<long unsigned int, int> chainIdDict;
    int i = 0;
    for (auto& [k, v] : matchedPtsUn){
        vPrevPtsUn.emplace_back(v.first);
        vCurrPtsUn.emplace_back(v.second);
        chainIdDict[k] = i;
        i++;
    }

    std::vector<uchar> status;
    cv::findFundamentalMat(vPrevPtsUn, vCurrPtsUn, cv::FM_RANSAC, 1, 0.99, status);

    // deal with current image data
    for(int i = 0; i < mCurrImageData.N; i++){
        auto chainId = mCurrImageData.mvChainIds[i];
        if(status[chainIdDict[chainId]] == 0){ // This point is rejected by F matrix, so it should be reset.
            mCurrImageData.mvChainIds[i] = -1;
            mCurrImageData.mvChainLens[i] = 1;
            mCurrImageData.mvPtUnOffsets[i] = cv::Point2f(0, 0);
        }
    }

    // deal with previous image data
    for(int i = 0; i < mPrevImageData.N; i++){
        auto chainId = mPrevImageData.mvChainIds[i];
        if(status[chainIdDict[chainId]] == 0){
            mPrevImageData.mvChainLens[i]--;
            if(mPrevImageData.mvChainLens[i] < 2){
                mPrevImageData.mvChainIds[i] = -1;
            }
        }
    }
}

cv::Mat FeatureTracker::DrawMatches(const std::string& winName){
    int w = mImgWidth;
    int h = mImgHeight;

    cv::Mat imgShow(h, w * 2, CV_8UC3, cv::Scalar::all(0));
    cv::Mat tmp, tmpUn;
    // if(mK.empty()){
        cv::cvtColor(mPrevImageData.mImg, tmp, cv::COLOR_GRAY2BGR);
        tmp.copyTo(imgShow(cv::Rect(0, 0, w, h)));
        cv::cvtColor(mCurrImageData.mImg, tmp, cv::COLOR_GRAY2BGR);
        tmp.copyTo(imgShow(cv::Rect(w, 0, w, h)));
    // }
    // else{
    //     cv::cvtColor(mPrevImageData.mImg, tmp, cv::COLOR_GRAY2BGR);
    //     cv::undistort(tmp, tmpUn, mK, mDistCoef, mK);
    //     tmpUn.copyTo(imgShow(cv::Rect(0, 0, w, h)));

    //     cv::cvtColor(mCurrImageData.mImg, tmp, cv::COLOR_GRAY2BGR);
    //     cv::undistort(tmp, tmpUn, mK, mDistCoef, mK);
    //     tmpUn.copyTo(imgShow(cv::Rect(w, 0, w, h)));
    // }

    std::unordered_map<int, std::pair<cv::Point2f, cv::Point2f>> matchedPts, matchedPtsUn;
    for(int i = 0; i < mCurrImageData.N; i++){
        if(mCurrImageData.mvChainLens[i] >= 2){
            matchedPts[mCurrImageData.mvChainIds[i]] = std::make_pair(cv::Point2f(), mCurrImageData.mvPts[i]);
            matchedPtsUn[mCurrImageData.mvChainIds[i]] = std::make_pair(cv::Point2f(), mCurrImageData.mvPtsUn[i]);
        }
        cv::circle(imgShow, (mCurrImageData.mvPts[i] + cv::Point2f(w, 0)), 3, cv::Scalar(0, 255, 0));
    }

    for(int i = 0; i < mPrevImageData.N; i++){
        if(matchedPts.find(mPrevImageData.mvChainIds[i]) != matchedPts.end()){
            matchedPts[mPrevImageData.mvChainIds[i]].first = mPrevImageData.mvPts[i];
            matchedPtsUn[mPrevImageData.mvChainIds[i]].first = mPrevImageData.mvPtsUn[i];
        }
        cv::circle(imgShow, mPrevImageData.mvPts[i], 3, cv::Scalar(0, 255, 0));
    }
    int trackedNum = matchedPts.size();

    for (auto& [k, v] : matchedPts){
        cv::circle(imgShow, v.first, 3, cv::Scalar(0, 255, 0));
        cv::circle(imgShow, (v.second + cv::Point2f(w, 0)), 3, cv::Scalar(0, 255, 0));
        cv::line(imgShow, v.first, (v.second + cv::Point2f(w, 0)), cv::Scalar(0, 0, 255));
    }
    cv::putText(imgShow, std::to_string(trackedNum), cv::Point2i(20, 20), 1, 1, cv::Scalar(0, 0, 255));

    // cv::imshow(winName, imgShow);
    // cv::waitKey(10);
    return imgShow;
}

} // namespace Naive_SLAM_ROS
