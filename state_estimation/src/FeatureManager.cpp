//
// Created by hanfuyong on 2022/10/21
//

#include "FeatureManager.h"

namespace Naive_SLAM_ROS{

Feature::Feature(const Feature& feature):
mChainId(feature.mChainId), mFrameId(feature.mFrameId), mPtUn(feature.mPtUn), mPtUnOffset(feature.mPtUnOffset){
}

Feature::Feature(unsigned long chainId, unsigned long frameId, 
    const Eigen::Vector2d& ptUn, const Eigen::Vector2d& ptUnOffset):
mChainId(chainId), mFrameId(frameId), mPtUn(ptUn), mPtUnOffset(ptUnOffset){
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

FeatureChain::FeatureChain(unsigned long chainId, int windowSize, int startId):
mChainId(chainId), mWindowSize(windowSize), mvFeatures(std::vector<Feature>()),
mStartIdx(startId), mWorldPos(Eigen::Vector3d(0, 0, 0)), mbGood(false), mbPosSet(false){
}

FeatureChain::FeatureChain(const FeatureChain& featureChain):
mChainId(featureChain.mChainId), mWindowSize(featureChain.mWindowSize), 
mvFeatures(featureChain.mvFeatures), mStartIdx(featureChain.mStartIdx),
mWorldPos(featureChain.mWorldPos), mbGood(featureChain.mbGood), mbPosSet(featureChain.mbPosSet){
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

Eigen::Vector2d FeatureChain::GetOffset() const{
    return mvFeatures.back().mPtUn - mvFeatures.front().mPtUn;
}

void FeatureChain::EraseFront() {
    if(mStartIdx == 0){
        if(mvFeatures.size() > 1)
            mvFeatures.erase(mvFeatures.begin());
        else{
            mvFeatures.clear();
        }
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

void FeatureManager::Manage(const PointCloud& pc, unsigned long frameId, int startId){
    if(startId == 0){ // If this is the first pc in mqPointClouds, then nothing to be managed.
                      // This only should happend when system just start or need relocalize;
        return;
    }
    for(int i = 0; i < pc.mvPtsUn.size(); i++){
        auto chainId = pc.mvChainIds[i];
        Feature feat(chainId, frameId, pc.mvPtsUn[i], pc.mvPtUnOffsets[i]);
        
        if(mmChains.find(chainId) != mmChains.end()){ // This chainId already exist, just add feature
            mmChains.find(chainId)->second.AddFeature(feat);
        }
        else{ // This ChainId not exist. Need create a new one; Also need create a former feature
            FeatureChain featureChain(chainId, mWindowSize, startId - 1);

            Eigen::Vector2d ptUnFormer = pc.mvPtsUn[i] - pc.mvPtUnOffsets[i];
            Eigen::Vector2d ptUnOffsetsFormer(0, 0);
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

const std::unordered_map<unsigned long, FeatureChain>& FeatureManager::GetChains() const{
    return mmChains;
}

void FeatureManager::EraseFront() {
    auto iter = mmChains.begin();
    while(iter != mmChains.end()){
        iter->second.EraseFront();
        if(iter->second.GetChainLen() == 0){
            mmChains.erase(iter++);
        }
        else{
            iter++;
        }
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
        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& vMatches) const{

    for(auto& [chainId, chain] : mmChains){
        if(chain.mStartIdx > pos1 || chain.mStartIdx + chain.GetChainLen() <= pos2)
            continue;
        Eigen::Vector2d pt1 = chain.mvFeatures[pos1 - chain.mStartIdx].mPtUn;
        Eigen::Vector2d pt2 = chain.mvFeatures[pos2 - chain.mStartIdx].mPtUn;
        vMatches.emplace_back(std::make_pair(pt1, pt2));
    }
    return vMatches.size();
}

int FeatureManager::GetMatches(int pos1, int pos2, 
        std::vector<Eigen::Vector2d>& vPts1, std::vector<Eigen::Vector2d>& vPts2, 
        std::vector<unsigned long>& vChainIds) const{
    if(mmChains.empty()){
        std::cout << "No Chain exists!" << std::endl;
        return -1;
    }

    for(auto& [chainId, chain] : mmChains){
        if(chain.mStartIdx > pos1 || chain.mStartIdx + chain.GetChainLen() <= pos2)
            continue;
        Eigen::Vector2d pt1 = chain.mvFeatures[pos1 - chain.mStartIdx].mPtUn;
        Eigen::Vector2d pt2 = chain.mvFeatures[pos2 - chain.mStartIdx].mPtUn;
        vPts1.emplace_back(pt1);
        vPts2.emplace_back(pt2);
        vChainIds.emplace_back(chainId);
    }
    return vPts1.size();
}

int FeatureManager::GetMatches(int pos1, int pos2, std::vector<Eigen::Vector3d>& vPts3D, std::vector<Eigen::Vector2d>& vPts2D, 
        std::vector<Eigen::Vector2d>& vPts1, std::vector<Eigen::Vector2d>& vPts2, std::vector<unsigned long>& vChainIds) const{
    if(mmChains.empty()){
        std::cout << "No Chain exists!" << std::endl;
        return -1;
    }
    for(auto& [chainId, chain] : mmChains){
        if(chain.mStartIdx > pos1 || chain.mStartIdx + chain.GetChainLen() <= pos2)
            continue;
        Eigen::Vector2d pt1 = chain.mvFeatures[pos1 - chain.mStartIdx].mPtUn;
        Eigen::Vector2d pt2 = chain.mvFeatures[pos2 - chain.mStartIdx].mPtUn;
        if(chain.mbPosSet){
            vPts3D.emplace_back(chain.mWorldPos);
            vPts2D.emplace_back(pt2);
        }
        else{
            vPts1.emplace_back(pt1);
            vPts2.emplace_back(pt2);
            vChainIds.emplace_back(chainId);
        }
    }
    return vPts3D.size() + vPts1.size();
}

void FeatureManager::SetWorldPos(unsigned long chainId, const Eigen::Vector3d& pos){
    if(mmChains.find(chainId) != mmChains.end()){
        mmChains.find(chainId)->second.mWorldPos = pos;
        mmChains.find(chainId)->second.mbPosSet = true;
    }
    else{
        std::cout << "ChainId not found" << std::endl;
    }
}

void FeatureManager::UpdateWorldPos(unsigned long chainId, const Eigen::Vector3d& pos){
    if(mmChains.find(chainId) != mmChains.end()){
        mmChains.find(chainId)->second.mWorldPos = pos;
    }
}

void FeatureManager::SetChainGood(unsigned long chainId, bool bGood){
    if(mmChains.find(chainId) != mmChains.end()){
        mmChains.find(chainId)->second.mbGood = bGood;
    }
}

bool FeatureManager::IsChainGood(unsigned long chainId){
    if(mmChains.find(chainId) != mmChains.end()){
        return mmChains.find(chainId)->second.mbGood;
    }
    else{
        std::cout << "Error: This chain does not exist!" << std::endl;
        return false;
    }
}

bool FeatureManager::IsChainPosSet(unsigned long chainId){
    if(mmChains.find(chainId) != mmChains.end()){
        return mmChains.find(chainId)->second.mbPosSet;
    }
    else{
        std::cout << "Error: This chain does not exist!" << std::endl;
        return false;
    }
}

} // namespace Naive_SLAM_ROS