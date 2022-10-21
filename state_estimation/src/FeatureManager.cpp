//
// Created by hanfuyong on 2022/10/21
//

#include "FeatureManager.h"

namespace Naive_SLAM_ROS{

Feature::Feature(long unsigned int chainId, long unsigned int frameId, const cv::Vec3f& ptUn, const cv::Vec2f& ptUnOffset):
mChainId(chainId), mFrameId(frameId), mPtUn(ptUn), mPtUnOffset(ptUnOffset){
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

FeatureChain::FeatureChain(long unsigned int chainId):
mChainId(chainId){
}

FeatureChain::FeatureChain(const FeatureChain& featureChain):
mChainId(featureChain.mChainId), mvFeatures(featureChain.mvFeatures){
}

void FeatureChain::AddFeature(const Feature& feat){
    if(feat.mChainId != mChainId){
        std::cout << "Chain Id not match!" << std::endl;
        exit(0);
    }
    mvFeatures.emplace_back(feat);
}

int FeatureChain::GetChainLen() const {
    return mvFeatures.size();
}

cv::Vec2f FeatureChain::GetOffset() const{
    cv::Vec3f tmp = mvFeatures.back().mPtUn - mvFeatures.front().mPtUn;
    return {tmp[0], tmp[1]};
}

void FeatureChain::EraseFront(long unsigned int frameId) {
    auto first = mvFeatures.begin();
    if(frameId == first->mFrameId)
        mvFeatures.erase(first);
}

void FeatureChain::EraseBack(long unsigned int frameId) {
    auto last = mvFeatures.end() - 1;
    if(frameId == last->mFrameId)
        mvFeatures.erase(last);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

FeatureManager::FeatureManager(){
}

void FeatureManager::Manage(const Frame& frame, long unsigned int frameId){
    for(int i = 0; i < frame.mvPtsUn.size(); i++){
        auto chainId = frame.mvChainIds[i];
        Feature feat(chainId, frameId, frame.mvPtsUn[i], frame.mvPtUnOffsets[i]);
        if(mmChains.find(chainId) != mmChains.end()){
            mmChains[chainId].AddFeature(feat);
        }
        else{
            FeatureChain featureChain(chainId);
            featureChain.AddFeature(feat);
            mmChains.insert(std::make_pair(chainId, featureChain));
        }
    }
}
    
int FeatureManager::GetChains(int chainLen, std::vector<FeatureChain>& vChains) const {
    for(auto& [chainId, chain] : mmChains){
        if(chain.GetChainLen() == chainLen){
            vChains.emplace_back(chain);
        }
    }
    return vChains.size();
}

void FeatureManager::EraseFront(long unsigned int frameId) {
    for(auto& [chainId, chain] : mmChains){
        chain.EraseFront(frameId);
        if(chain.GetChainLen() == 0)
            mmChains.erase(chainId);
    }
}

void FeatureManager::EraseBack(long unsigned int frameId) {
    for(auto& [chainId, chain] : mmChains){
        chain.EraseBack(frameId);
        if(chain.GetChainLen() == 0)
            mmChains.erase(chainId);
    }
}

} // namespace Naive_SLAM_ROS
