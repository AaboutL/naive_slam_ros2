//
// Created by hanfuyong on 2022/10/20
//

#include "StateEstimator.h"

namespace Naive_SLAM_ROS{

StateEstimator::StateEstimator(const std::string& strParamFile):
mFrameId(0), mState(INITS1){
    cv::FileStorage fs(strParamFile.c_str(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "[Estimator] Param file not exist..." << std::endl;
        exit(0);
    }
    mWindowSize = fs["WindowSize"];
    mK = cv::Mat::eye(3, 3, CV_32FC1);
    mK.at<float>(0, 0) = fs["Camera.fx"];
    mK.at<float>(1, 1) = fs["Camera.fy"];
    mK.at<float>(0, 2) = fs["Camera.cx"];
    mK.at<float>(1, 2) = fs["Camera.cy"];
    mpFM = new FeatureManager(mWindowSize);
    mpInitializer = new Initializer(20, 18, mK);
}

void StateEstimator::Estimate(const std::pair<Frame, std::vector<IMU>>& meas){
    Frame frame = meas.first;
    mvFrames.emplace_back(frame);
    mpFM->Manage(frame, mFrameId, mvFrames.size() - 1);
    mFrameId++;
    if(mvFrames.size() == 1) // first frame
        return;
    if(mState == INITS1 || mState == INITS2){
        int flag = VisualInit();
        if(flag == 1){
            mState = INITS2;
        }
        if(flag == 2){
            mState = VIINIT;
        }
    }
    if(mState == VIINIT){
        // TODO
    }
}

int StateEstimator::VisualInit(){
    if(mState == INITS1){
        std::vector<cv::Vec2f> vPtsUn1, vPtsUn2;
        std::vector<unsigned long> vChainIds;
        std::vector<cv::Vec3f> vPts3D;
        bool bS1 = mpInitializer->VisualInitS1(mvFrames, mpFM, vPts3D, vPtsUn1, vPtsUn2, vChainIds);
        if(!bS1)
            return -1;
        Optimizer::VisualInitBA(mvFrames.front(), mvFrames.back(), mpFM, vPts3D, vPtsUn1, vPtsUn2, vChainIds, mK);
        return 1;
    }
    if(mState == INITS2 && mvFrames.size() == mWindowSize){
        bool bS2 = mpInitializer->VisualInitS2(mvFrames, mpFM);
        if(!bS2)
            return -1;
        Optimizer::VisualBA(mvFrames, mpFM);
        return 2;
    }
    return 0;
}
    
} // namespace Naive_SLAM_ROS
