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
        mPrevImageData = ImageData(mCurrImageData);
    }

    vChainIds = mCurrImageData.mvChainIds;
    vPtsUn = mCurrImageData.mvPtsUn;
    vPts = mCurrImageData.mvPts;
    vChainLens = mCurrImageData.mvChainLens;
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
    }

    for(int i = 0; i < mPrevImageData.N; i++){
        if(matchedPts.find(mPrevImageData.mvChainIds[i]) != matchedPts.end()){
            matchedPts[mPrevImageData.mvChainIds[i]].first = mPrevImageData.mvPts[i];
            matchedPtsUn[mPrevImageData.mvChainIds[i]].first = mPrevImageData.mvPtsUn[i];
        }
    }
    int beforeNum = matchedPts.size();

    for (auto& [k, v] : matchedPts){
        cv::circle(imgShow, v.first, 3, cv::Scalar(0, 255, 0));
        cv::circle(imgShow, (v.second + cv::Point2f(w, 0)), 3, cv::Scalar(0, 255, 0));
        cv::line(imgShow, v.first, (v.second + cv::Point2f(w, 0)), cv::Scalar(0, 0, 255));
    }

    std::vector<cv::Point2f> vPrevPtsUn, vCurrPtsUn, vPG, vCG;
    std::vector<int> ids;
    for (auto& [k, v] : matchedPtsUn){
        vPrevPtsUn.emplace_back(v.first);
        vCurrPtsUn.emplace_back(v.second);
        ids.push_back(k);
    }

    std::vector<uchar> status;
    cv::findFundamentalMat(vPrevPtsUn, vCurrPtsUn, cv::FM_RANSAC, 1, 0.99, status);
    for(int i = 0; i < status.size(); i++){
        if(status[i] == 1){
            vPG.emplace_back(matchedPts[ids[i]].first);
            vCG.emplace_back(matchedPts[ids[i]].second);
        }
    }
    int afterNum = vPG.size();

    for(int i = 0; i < vPG.size(); i++){
        cv::circle(imgShow, vPG[i], 3, cv::Scalar(0, 255, 0));
        cv::circle(imgShow, (vCG[i] + cv::Point2f(w, 0)), 3, cv::Scalar(0, 255, 0));
        cv::line(imgShow, vPG[i], (vCG[i] + cv::Point2f(w, 0)), cv::Scalar(255, 0, 0));
    }
    cv::putText(imgShow, std::to_string(beforeNum), cv::Point2i(20, 20), 1, 1, cv::Scalar(0, 0, 255));
    cv::putText(imgShow, std::to_string(afterNum), cv::Point2i(20, 30), 1, 1, cv::Scalar(0, 0, 255));

    // cv::imshow(winName, imgShow);
    // cv::waitKey(10);
    return imgShow;
}

} // namespace Naive_SLAM_ROS
