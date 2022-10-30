//
// Created by hanfuyong on 2022/10/27
//

#include "Initializer.h"
#include "GeometryFunc.h"

namespace Naive_SLAM_ROS{

Initializer::Initializer(int matchNumTh, float parallaxTh, const Eigen::Matrix3d& K, FeatureManager* pFM):
mMatchNumTh(matchNumTh), mParallaxTh(parallaxTh), mK(K), mInitIdx(0), mpFM(pFM){
}


bool Initializer::VisualInitS1(std::vector<Frame>& vFrames, std::vector<Eigen::Vector3d>& vPts3D, 
        std::vector<Eigen::Vector2d>& vPts2D1, std::vector<Eigen::Vector2d>& vPts2D2,
        std::vector<unsigned long>& vChainIds){

    auto frameNum = vFrames.size();
    int matchNum = mpFM->GetMatches(0, frameNum-1, vPts2D1, vPts2D2, vChainIds);

    std::vector<cv::Vec2f> cvPts2D1(matchNum), cvPts2D2(matchNum);

    std::vector<float> parallaxs;
    for(int i = 0; i < matchNum; i++){
        Eigen::Vector2d chainOffset = vPts2D2[i] - vPts2D1[i];
        float parallax = std::sqrt(chainOffset.dot(chainOffset));
        parallaxs.push_back(parallax);
        cvPts2D1[i] = cv::Vec2f(vPts2D1[i][0], vPts2D1[i][1]);
        cvPts2D2[i] = cv::Vec2f(vPts2D2[i][0], vPts2D2[i][1]);
    }
    std::sort(parallaxs.begin(), parallaxs.end());
    if(matchNum < 20 || parallaxs[matchNum / 2] < 18)
        return false;

    cv::Mat mask, cvR21, cvt21, K;
    cv::eigen2cv(mK, K);
    cv::Mat EMat = cv::findEssentialMat(cvPts2D1, cvPts2D2, K, cv::RANSAC, 0.999, 3.84, mask);
    int inlier_cnt = cv::recoverPose(EMat, cvPts2D1, cvPts2D2, K, cvR21, cvt21, mask);
    Eigen::Matrix3d R21 = TypeConverter::MatCVtoEigen(cvR21);
    Eigen::Vector3d t21 = TypeConverter::VecCVtoEigen(cvt21);
    vFrames.back().SetTcw(R21, t21);

    vPts3D = TriangulateTwoFrame(vFrames.front().mRcw, vFrames.front().mtcw, 
        vFrames.back().mRcw, vFrames.back().mtcw, vPts2D1, vPts2D2, vChainIds);

    mInitIdx = frameNum - 1;
    return true;
}
    
bool Initializer::VisualInitS2(std::vector<Frame>& vFrames){
    // first deal with frames between 0 and mInitIdx
    for(int i = 1; i < mInitIdx; i++){

        std::vector<Eigen::Vector3d> vPts3D;
        std::vector<Eigen::Vector2d> vPts2D, vPts1, vPts2;
        std::vector<unsigned long> vChainIds;
        int matchNum = mpFM->GetMatches(0, i, vPts3D, vPts2D, vPts1, vPts2, vChainIds);

        Eigen::Matrix3d Rcw2;
        Eigen::Vector3d tcw2;
        GeometryFunc::SolvePnP(vPts3D, vPts2D, mK, Rcw2, tcw2);
        vFrames[i].SetTcw(Rcw2, tcw2);

        TriangulateTwoFrame(vFrames[0].mRcw, vFrames[0].mtcw, 
            vFrames[i].mRcw, vFrames[i].mtcw, vPts1, vPts2, vChainIds);
    }

    // second deal with frames between mInitIdx and mWindowSize - 1
    for(int i = mInitIdx + 1; i < vFrames.size(); i++){
        std::vector<Eigen::Vector3d> vPts3D;
        std::vector<Eigen::Vector2d> vPts2D, vPts1, vPts2;
        std::vector<unsigned long> vChainIds;
        int matchNum = mpFM->GetMatches(i - 1, i, vPts3D, vPts2D, vPts1, vPts2, vChainIds);

        Eigen::Matrix3d Rcw2;
        Eigen::Vector3d tcw2;
        GeometryFunc::SolvePnP(vPts3D, vPts2D, mK, Rcw2, tcw2);
        vFrames[i].SetTcw(Rcw2, tcw2);

        TriangulateTwoFrame(vFrames[i - 1].mRcw, vFrames[i - 1].mtcw, 
            vFrames[i].mRcw, vFrames[i].mtcw, vPts1, vPts2, vChainIds);
    }

    // third: deal with all other chains from 1 to mInitIdx
    for(int i = 2; i <= mInitIdx; i++){
        std::vector<Eigen::Vector3d> vPts3D;
        std::vector<Eigen::Vector2d> vPts2D, vPts1, vPts2;
        std::vector<unsigned long> vChainIds;
        int matchNum = mpFM->GetMatches(i - 1, i, vPts3D, vPts2D, vPts1, vPts2, vChainIds);

        TriangulateTwoFrame(vFrames[i - 1].mRcw, vFrames[i - 1].mtcw, 
            vFrames[i].mRcw, vFrames[i].mtcw, vPts1, vPts2, vChainIds);
    }

    return true;
}

std::vector<Eigen::Vector3d> Initializer::TriangulateTwoFrame(const Eigen::Matrix3d& Rcw1, const Eigen::Vector3d& tcw1,
        const Eigen::Matrix3d& Rcw2, const Eigen::Vector3d& tcw2,
        const std::vector<Eigen::Vector2d>& vPts2D1, const std::vector<Eigen::Vector2d>& vPts2D2,
        const std::vector<unsigned long>& vChainIds){

    std::vector<Eigen::Vector3d> vPts3D(vPts2D1.size(), Eigen::Vector3d(0, 0, 0));
    Eigen::Matrix<double, 3, 4> P1, P2;
    P1.block<3, 3>(0, 0) = Rcw1;
    P1.block<3, 1>(0, 3) = tcw1;
    P1 = mK * P1;
    P2.block<3, 3>(0, 0) = Rcw2;
    P2.block<3, 1>(0, 3) = tcw2;
    P2 = mK * P2;

    int rej_z1 = 0;
    int rej_z2 = 0;
    int rej_err1 = 0;
    int rej_err2 = 0;
    for(int j = 0; j < vPts2D1.size(); j++){
        Eigen::Vector2d pt1 = vPts2D1[j], pt2 = vPts2D2[j];
        Eigen::Vector3d pt3DW = GeometryFunc::Triangulate(pt1, pt2, P1, P2);

        auto pt3DC1 = Rcw1 * pt3DW + tcw1;
        if(!finite(pt3DC1[0]) || !finite(pt3DC1[1]) || !finite(pt3DC1[2]) || pt3DC1[2] <= 0){
            rej_z1++;
            continue;
        }

        auto pt3DC2 = Rcw2 * pt3DW + tcw2;
        if(!finite(pt3DC2[0]) || !finite(pt3DC2[1]) || !finite(pt3DC2[2]) || pt3DC2[2] <= 0){
            rej_z2++;
            continue;
        }

        Eigen::Vector2d uv1 = GeometryFunc::project(pt3DC1, mK);
        auto err1 = uv1 - pt1;
        auto err12 = err1.dot(err1);
        if(err12 > 5.991){
            rej_err1++;
            continue;
        }

        Eigen::Vector2d uv2 = GeometryFunc::project(pt3DC2, mK);
        auto err2 = uv2 - pt2;
        auto err22 = err2.dot(err2);
        if(err22 > 5.991){
            rej_err2++;
            continue;
        }

        vPts3D[j] = pt3DW;
        mpFM->SetWorldPos(vChainIds[j], pt3DW);
    }

    return vPts3D;
}

} // namespace Naive_SLAM_ROS
