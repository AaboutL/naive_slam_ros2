//
// Created by hanfuyong on 2022/10/21
//

#include "FeatureManager.h"

namespace Naive_SLAM_ROS{

Feature::Feature(const Feature& feature):
mChainId(feature.mChainId), mFrameId(feature.mFrameId), mPtUn(feature.mPtUn), mPtUnOffset(feature.mPtUnOffset){
}

Feature::Feature(unsigned long chainId, unsigned long frameId, 
    const cv::Vec3f& ptUn, const cv::Vec2f& ptUnOffset):
mChainId(chainId), mFrameId(frameId), mPtUn(ptUn), mPtUnOffset(ptUnOffset){
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

FeatureChain::FeatureChain(unsigned long chainId, int windowSize, int startId):
mChainId(chainId), mWindowSize(windowSize), mvFeatures(std::vector<Feature>()),
mStartIdx(startId), mWorldPos(cv::Vec3f(0, 0, 0)), mbOptimized(false){
}

FeatureChain::FeatureChain(const FeatureChain& featureChain):
mChainId(featureChain.mChainId), mWindowSize(featureChain.mWindowSize), 
mvFeatures(featureChain.mvFeatures), mStartIdx(featureChain.mStartIdx),
mWorldPos(featureChain.mWorldPos), mbOptimized(featureChain.mbOptimized){
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

void FeatureChain::EraseFront() {
    if(mStartIdx == 0){
        auto first = mvFeatures.begin();
        mvFeatures.erase(first);
    }
    else{
        mStartIdx--;
    }
}

void FeatureChain::EraseBack() {
    if(mStartIdx + mvFeatures.size() == mWindowSize + 1){
        auto last = mvFeatures.end() - 1;
        mvFeatures.erase(last);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

FeatureManager::FeatureManager(int windowSize):
mWindowSize(windowSize){
}

void FeatureManager::Manage(const Frame& frame, unsigned long frameId, int startId){
    if(startId == 0){ // If this is the first frame in mqFrames, then nothing to be managed.
                      // This only should happend when system just start or need relocalize;
        return;
    }
    for(int i = 0; i < frame.mvPtsUn.size(); i++){
        auto chainId = frame.mvChainIds[i];
        Feature feat(chainId, frameId, frame.mvPtsUn[i], frame.mvPtUnOffsets[i]);
        
        if(mmChains.find(chainId) != mmChains.end()){ // This chainId already exist, just add feature
            mmChains.find(chainId)->second.AddFeature(feat);
        }
        else{ // This ChainId not exist. Need create a new one; Also need create a former feature
            FeatureChain featureChain(chainId, mWindowSize, startId - 1);

            cv::Vec3f ptUnFormer(frame.mvPtsUn[i][0] - frame.mvPtUnOffsets[i][0],
                                  frame.mvPtsUn[i][1] - frame.mvPtUnOffsets[i][1], 1);
            cv::Vec2f ptUnOffsetsFormer(0, 0);
            Feature featFormer(chainId, frameId - 1, ptUnFormer, ptUnOffsetsFormer);
            featureChain.AddFeature(featFormer);
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

void FeatureManager::EraseFront() {
    for(auto& [chainId, chain] : mmChains){
        chain.EraseFront();
        if(chain.GetChainLen() == 0)
            mmChains.erase(chainId);
    }
}

void FeatureManager::EraseBack() {
    for(auto& [chainId, chain] : mmChains){
        chain.EraseBack();
        if(chain.GetChainLen() == 0)
            mmChains.erase(chainId);
    }
}

int FeatureManager::GetMatches(int pos1, int pos2, 
        std::vector<std::pair<cv::Vec2f, cv::Vec2f>>& vMatches) const{

    for(auto& [chainId, chain] : mmChains){
        if(chain.mStartIdx > pos1 || chain.mStartIdx + chain.GetChainLen() <= pos2)
            continue;
        cv::Vec3f pt1 = chain.mvFeatures[pos1 - chain.mStartIdx].mPtUn;
        cv::Vec3f pt2 = chain.mvFeatures[pos2 - chain.mStartIdx].mPtUn;
        vMatches.emplace_back(std::make_pair(cv::Vec2f(pt1[0], pt1[1]), cv::Vec2f(pt2[0], pt2[1])));
    }
    return vMatches.size();
}

int FeatureManager::GetMatches(int pos1, int pos2, 
        std::vector<cv::Vec2f>& vPts1, std::vector<cv::Vec2f>& vPts2, 
        std::vector<unsigned long>& vChainIds) const{
    if(mmChains.empty()){
        std::cout << "No Chain exists!" << std::endl;
        return -1;
    }

    for(auto& [chainId, chain] : mmChains){
        if(chain.mStartIdx > pos1 || chain.mStartIdx + chain.GetChainLen() <= pos2)
            continue;
        cv::Vec3f pt1 = chain.mvFeatures[pos1 - chain.mStartIdx].mPtUn;
        cv::Vec3f pt2 = chain.mvFeatures[pos2 - chain.mStartIdx].mPtUn;
        vPts1.emplace_back(cv::Vec2f(pt1[0], pt1[1]));
        vPts2.emplace_back(cv::Vec2f(pt2[0], pt2[1]));
        vChainIds.emplace_back(chainId);
    }
    return vPts1.size();
}

int FeatureManager::GetMatches(int pos1, int pos2, std::vector<cv::Vec3f>& vPts3D, std::vector<cv::Vec2f>& vPts2D, 
        std::vector<cv::Vec2f>& vPts1, std::vector<cv::Vec2f>& vPts2, std::vector<unsigned long>& vChainIds) const{
    if(mmChains.empty()){
        std::cout << "No Chain exists!" << std::endl;
        return -1;
    }
    for(auto& [chainId, chain] : mmChains){
        if(chain.mStartIdx > pos1 || chain.mStartIdx + chain.GetChainLen() <= pos2)
            continue;
        cv::Vec3f pt1 = chain.mvFeatures[pos1 - chain.mStartIdx].mPtUn;
        cv::Vec3f pt2 = chain.mvFeatures[pos2 - chain.mStartIdx].mPtUn;
        if(chain.mbOptimized){
            vPts3D.emplace_back(chain.mWorldPos);
            vPts2D.emplace_back(cv::Vec2f(pt2[0], pt2[1]));
        }
        else{
            vPts1.emplace_back(cv::Vec2f(pt1[0], pt1[1]));
            vPts2.emplace_back(cv::Vec2f(pt2[0], pt2[1]));
            vChainIds.emplace_back(chainId);
        }
    }
    return vPts3D.size() + vPts1.size();
}

void FeatureManager::SetChainPosition(unsigned long chainId, const cv::Vec3f& pos){
    if(mmChains.find(chainId) != mmChains.end()){
        mmChains.find(chainId)->second.mWorldPos = pos;
    }
}

void FeatureManager::SetChainOptFlag(unsigned long chainId, bool bOpted){
    if(mmChains.find(chainId) != mmChains.end()){
        mmChains.find(chainId)->second.mbOptimized = bOpted;
    }
}

} // namespace Naive_SLAM_ROS
