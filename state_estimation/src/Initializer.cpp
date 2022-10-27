//
// Created by hanfuyong on 2022/10/27
//

#include "Initializer.h"
#include "GeometryFunc.h"

namespace Naive_SLAM_ROS{

Initializer::Initializer(int matchNumTh, float parallaxTh, const cv::Mat& K):
mMatchNumTh(matchNumTh), mParallaxTh(parallaxTh), mK(K.clone()), mInitIdx(0){
}


bool Initializer::VisualInitS1(std::vector<Frame>& qFrames, FeatureManager* pFM,
        std::vector<cv::Vec3f>& vPts3D, std::vector<cv::Vec2f>& vPts2D1, std::vector<cv::Vec2f>& vPts2D2,
        std::vector<unsigned long>& vChainIds){

    auto frameNum = qFrames.size();
    int matchNum = pFM->GetMatches(0, frameNum-1, vPts2D1, vPts2D2, vChainIds);

    std::vector<float> parallaxs;
    for(int i = 0; i < matchNum; i++){
        cv::Vec2f chainOffset = vPts2D2[i] - vPts2D1[i];
        float parallax = std::sqrt(chainOffset[0] * chainOffset[0] + chainOffset[1] * chainOffset[1]);
        parallaxs.push_back(parallax);
    }
    std::sort(parallaxs.begin(), parallaxs.end());
    if(matchNum < 20 || parallaxs[matchNum / 2] < 18)
        return false;

    cv::Mat mask, R21, t21;
    cv::Mat EMat = cv::findEssentialMat(vPts2D1, vPts2D2, mK, cv::RANSAC, 0.999, 3.84, mask);
    int inlier_cnt = cv::recoverPose(EMat, vPts2D1, vPts2D2, mK, R21, t21, mask);
    R21.convertTo(R21, CV_32F);
    t21.convertTo(t21, CV_32F);
    qFrames.back().SetTcw(R21, t21);

    cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
    mK.copyTo(P1.rowRange(0, 3).colRange(0, 3));
    cv::Mat P2(3, 4, CV_32F, cv::Scalar(0));
    R21.copyTo(P2.rowRange(0, 3).colRange(0, 3));
    t21.copyTo(P2.rowRange(0, 3).col(3));
    P2 = mK * P2;

    vPts3D.resize(matchNum, cv::Vec3f(0, 0, 0));
    for(int i = 0; i < matchNum; i++){
        cv::Vec2f pt1 = vPts2D1[i], pt2 = vPts2D2[i];
        cv::Vec3f pt3DC1 = GeometryFunc::Triangulate(pt1, pt2, P1, P2);

        if(!isfinite(pt3DC1[0]) || !isfinite(pt3DC1[1] || !isfinite(pt3DC1[2])) || pt3DC1[2] <= 0)
            continue;

        cv::Vec2f uv1 = GeometryFunc::project(pt3DC1, mK);
        float squareErr1 = (uv1[0] - pt1[0]) * (uv1[0] - pt1[0]) + (uv1[1] - pt1[1]) * (uv1[1] - pt1[1]);
        if(squareErr1 > 4)
            continue;
        
        cv::Mat pt3DC2 = R21 * pt3DC1 + t21;
        if(pt3DC2.at<float>(2) <= 0)
            continue;
        cv::Vec2f uv2 = GeometryFunc::project(pt3DC2, mK);
        float squareErr2 = (uv2[0] - pt2[0]) * (uv2[0] - pt2[0]) + (uv2[1] - pt2[1]) * (uv2[1] - pt2[1]);
        if(squareErr2 > 4)
            continue;
        vPts3D[i] = cv::Vec3f(pt3DC1[0], pt3DC1[1], pt3DC1[2]);
    }
    mInitIdx = frameNum - 1;
    return true;
}
    
bool Initializer::VisualInitS2(std::vector<Frame>& qFrames, FeatureManager* pFM){
    // first deal with frame between 0 and mInitIdx
    for(int i = 1; i < mInitIdx; i++){
        std::vector<cv::Vec3f> vPts3D;
        std::vector<cv::Vec2f> vPts2D, vPts1, vPts2;
        std::vector<unsigned long> vChainIds;
        int matchNum = pFM->GetMatches(0, i, vPts3D, vPts2D, vPts1, vPts2, vChainIds);
        cv::Mat r21, t21, R21, inliers;
        cv::solvePnPRansac(vPts3D, vPts2D, mK, cv::Mat::zeros(4, 1, CV_32F),
                           r21, t21, false, 100, 2, 0.99, inliers, cv::SOLVEPNP_EPNP);
        cv::Rodrigues(r21, R21);
        R21.convertTo(R21, CV_32F);
        t21.convertTo(t21, CV_32F);
        qFrames[i].SetTcw(R21, t21);

        cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
        mK.copyTo(P1.rowRange(0, 3).colRange(0, 3));
        cv::Mat P2(3, 4, CV_32F, cv::Scalar(0));
        R21.copyTo(P2.rowRange(0, 3).colRange(0, 3));
        t21.copyTo(P2.rowRange(0, 3).col(3));
        P2 = mK * P2;
        for(int j = 0; j < vPts1.size(); j++){
            cv::Vec3f pt3DC1 = GeometryFunc::Triangulate(vPts1[j], vPts2[j], P1, P2);
            pFM->SetChainPosition(vChainIds[j], pt3DC1);
            pFM->SetChainOptFlag(vChainIds[j], true);
        }
    }
    // second deal with frame between mInitIdx and mWindowSize - 1
    for(int i = mInitIdx + 1; i < qFrames.size(); i++){
        std::vector<cv::Vec3f> vPts3D;
        std::vector<cv::Vec2f> vPts2D, vPts1, vPts2;
        std::vector<unsigned long> vChainIds;
        int matchNum = pFM->GetMatches(i - 1, i, vPts3D, vPts2D, vPts1, vPts2, vChainIds);
        cv::Mat r2w, t2w, R2w, inliers;
        cv::solvePnPRansac(vPts3D, vPts2D, mK, cv::Mat::zeros(4, 1, CV_32F),
                           r2w, t2w, false, 100, 2, 0.99, inliers, cv::SOLVEPNP_EPNP);
        cv::Rodrigues(r2w, R2w);
        R2w.convertTo(R2w, CV_32F);
        t2w.convertTo(t2w, CV_32F);
        qFrames[i].SetTcw(R2w, t2w);

        cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
        qFrames[i-1].mRcw.copyTo(P1.rowRange(0, 3).colRange(0, 3));
        qFrames[i-1].mtcw.copyTo(P1.rowRange(0, 3).col(3));
        P1 = mK * P1;

        cv::Mat P2(3, 4, CV_32F, cv::Scalar(0));
        R2w.copyTo(P2.rowRange(0, 3).colRange(0, 3));
        t2w.copyTo(P2.rowRange(0, 3).col(3));
        P2 = mK * P2;
        for(int j = 0; j < vPts1.size(); j++){
            cv::Vec3f pt3DW = GeometryFunc::Triangulate(vPts1[j], vPts2[j], P1, P2);
            pFM->SetChainPosition(vChainIds[j], pt3DW);
            pFM->SetChainOptFlag(vChainIds[j], true);
        }
    }
    // third: deal with all other chains from 1 to mInitIdx
    for(int i = 2; i <= mInitIdx; i++){
        std::vector<cv::Vec3f> vPts3D;
        std::vector<cv::Vec2f> vPts2D, vPts1, vPts2;
        std::vector<unsigned long> vChainIds;
        int matchNum = pFM->GetMatches(i - 1, i, vPts3D, vPts2D, vPts1, vPts2, vChainIds);

        cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
        qFrames[i-1].mRcw.copyTo(P1.rowRange(0, 3).colRange(0, 3));
        qFrames[i-1].mtcw.copyTo(P1.rowRange(0, 3).col(3));
        P1 = mK * P1;

        cv::Mat P2(3, 4, CV_32F, cv::Scalar(0));
        qFrames[i].mRcw.copyTo(P2.rowRange(0, 3).colRange(0, 3));
        qFrames[i].mtcw.copyTo(P2.rowRange(0, 3).col(3));
        P2 = mK * P2;

        for(int j = 0; j < vPts1.size(); j++){
            cv::Vec3f pt3DW = GeometryFunc::Triangulate(vPts1[j], vPts2[j], P1, P2);
            pFM->SetChainPosition(vChainIds[j], pt3DW);
            pFM->SetChainOptFlag(vChainIds[j], true);
        }
    }
    return true;
}

} // namespace Naive_SLAM_ROS
