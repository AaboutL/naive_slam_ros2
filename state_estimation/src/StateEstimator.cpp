//
// Created by hanfuyong on 2022/10/20
//

#include "StateEstimator.h"

namespace Naive_SLAM_ROS{

StateEstimator::StateEstimator(int windowSize):
mWindowSize(windowSize), mFrameId(0), mState(INIT){
    mpFM = new FeatureManager();

}

void StateEstimator::Estimate(const std::pair<Frame, std::vector<IMU>>& pMeas){
    Frame frame = pMeas.first;
    mqFrames.emplace_back(frame);
    mpFM->Manage(frame, mFrameId);
    if(mState == INIT){
        // TODO
    }
}

bool StateEstimator::VisualInit(){
    int frameNum = mqFrames.size();
    if(frameNum <= mWindowSize){
        std::vector<FeatureChain> vChains;
        int chainNum = mpFM->GetChains(frameNum, vChains);

        std::vector<double> parallaxs;
        for(int i = 0; i < chainNum; i++){
            cv::Vec2d chainOffset = vChains[i].GetOffset();
            double parallax = std::sqrt(chainOffset[0] * chainOffset[0] + chainOffset[1] * chainOffset[1]);
            parallaxs.push_back(parallax);
        }
        std::sort(parallaxs.begin(), parallaxs.end());
        if(chainNum < 30 || parallaxs[chainNum / 2] < 18)
            return;
        
        std::vector<cv::Vec3d> vPts1, vPts2;
        for(auto& chain: vChains){
            vPts1.emplace_back(chain.mvFeatures.front());
            vPts2.emplace_back(chain.mvFeatures.back());
        }
        // TODO: Fundamental Matrix
    }
    else{
        mqFrames.pop_front();
        mpFM->EraseFront(mFrameId - mWindowSize);
    }

}
    
} // namespace Naive_SLAM_ROS
