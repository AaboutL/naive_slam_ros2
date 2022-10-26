//
// Created by hanfuyong on 2022/10/20
//

#include "StateEstimator.h"

namespace Naive_SLAM_ROS{

StateEstimator::StateEstimator(const std::string& strParamFile):
mFrameId(0), mState(INIT){
    cv::FileStorage fs(strParamFile.c_str(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
        // std::cout << "[Estimator] Param file not exist..." << std::endl;
        exit(0);
    }
    mWindowSize = fs["WindowSize"];
    mK = cv::Mat::eye(3, 3, CV_32FC1);
    mK.at<float>(0, 0) = fs["Camera.fx"];
    mK.at<float>(1, 1) = fs["Camera.fy"];
    mK.at<float>(0, 2) = fs["Camera.cx"];
    mK.at<float>(1, 2) = fs["Camera.cy"];
    mpFM = new FeatureManager(mWindowSize);
}

StateEstimator::StateEstimator(int windowSize, const cv::Mat& K):
mWindowSize(windowSize), mFrameId(0), mState(INIT), mK(K.clone()){
    mpFM = new FeatureManager(windowSize);

}

void StateEstimator::Estimate(const std::pair<Frame, std::vector<IMU>>& meas){
    Frame frame = meas.first;
    mqFrames.emplace_back(frame);
    mpFM->Manage(frame, mFrameId, mqFrames.size() - 1);
    if(mState == INIT){
        VisualInit();
    }
}

bool StateEstimator::VisualInit(){
    int frameNum = mqFrames.size();
    std::cout << "test1: " << frameNum << "  " << mWindowSize << std::endl;
    
    if(frameNum <= mWindowSize){
        std::vector<cv::Vec2f> vPtsUn1, vPtsUn2;
        std::vector<unsigned long> vChainIds;
        int matchNum = mpFM->GetMatches(0, frameNum-1, vPtsUn1, vPtsUn2, vChainIds);

        std::vector<float> parallaxs;
        for(int i = 0; i < matchNum; i++){
            cv::Vec2f chainOffset = vPtsUn2[i] - vPtsUn1[i];
            float parallax = std::sqrt(chainOffset[0] * chainOffset[0] + chainOffset[1] * chainOffset[1]);
            parallaxs.push_back(parallax);
        }
        std::sort(parallaxs.begin(), parallaxs.end());
        std::cout << "matchNum=" << matchNum << "  mid parallax=" << parallaxs[matchNum / 2] << std::endl;
        if(matchNum < 30 || parallaxs[matchNum / 2] < 18)
            return false;

        cv::Mat mask, R21, t21;
        cv::Mat EMat = cv::findEssentialMat(vPtsUn1, vPtsUn2, mK, cv::RANSAC, 0.999, 3.84, mask);
        int inlier_cnt = cv::recoverPose(EMat, vPtsUn1, vPtsUn2, mK, R21, t21, mask);
        R21.convertTo(R21, CV_32F);
        t21.convertTo(t21, CV_32F);
        mqFrames.back().SetTcw(R21, t21);

        cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
        mK.copyTo(P1.rowRange(0, 3).colRange(0, 3));
        cv::Mat P2(3, 4, CV_32F, cv::Scalar(0));
        R21.copyTo(P2.rowRange(0, 3).colRange(0, 3));
        t21.copyTo(P2.rowRange(0, 3).col(3));
        P2 = mK * P2;

        std::cout << "test" << std::endl;
        std::vector<cv::Vec3f> vPts3D(matchNum, cv::Vec3f(0, 0, 0));
        for(int i = 0; i < matchNum; i++){
            cv::Mat pt3DC1;
            cv::Vec2f pt1 = vPtsUn1[i], pt2 = vPtsUn2[i];
            GeometryFunc::Triangulate(pt1, pt2, P1, P2, pt3DC1);
            if(!isfinite(pt3DC1.at<float>(0)) || !isfinite(pt3DC1.at<float>(1) || !isfinite(pt3DC1.at<float>(2))))
                continue;

            cv::Vec2f uv1 = GeometryFunc::project(pt3DC1, mK);
            float squareErr1 = (uv1[0] - pt1[0]) * (uv1[0] - pt1[0]) + (uv1[1] - pt1[1]) * (uv1[1] - pt1[1]);
            if(squareErr1 > 4)
                continue;
            
            cv::Mat pt3DC2 = R21 * pt3DC1 + t21;
            cv::Vec2f uv2 = GeometryFunc::project(pt3DC2, mK);
            float squareErr2 = (uv2[0] - pt2[0]) * (uv2[0] - pt2[0]) + (uv2[1] - pt2[1]) * (uv2[1] - pt2[1]);
            if(squareErr2 > 4)
                continue;
            vPts3D[i] = cv::Vec3f(pt3DC1.at<float>(0), pt3DC1.at<float>(1), pt3DC1.at<float>(2));
        }
        if(!mqFrames.front().mTcw.empty())
            std::cout << "front not empty" << std::endl;
        else 
            std::cout << "front empty" << std::endl;
        Optimizer::VisualInitBA(mqFrames.front(), mqFrames.back(), mpFM, vPts3D, vPtsUn1, vPtsUn2, vChainIds, mK);
        return true;
    }
    // else{
    // }
}
    
} // namespace Naive_SLAM_ROS
