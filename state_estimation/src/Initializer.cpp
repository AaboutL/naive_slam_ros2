//
// Created by hanfuyong on 2022/10/27
//

#include "Initializer.h"
#include "GeometryFunc.h"

namespace Naive_SLAM_ROS{

Initializer::Initializer(int matchNumTh, float parallaxTh, const Eigen::Matrix3d& K, FeatureManager* pFM,
                        Sophus::SE3d& Tbc):
mMatchNumTh(matchNumTh), mParallaxTh(parallaxTh), mK(K), mInitIdx(0), mpFM(pFM), 
mTbc(Tbc), mTcb(Tbc.inverse()){
}

void Initializer::Reset(){
    mInitIdx = 0;
}

bool Initializer::VisualOnlyInitS1(std::vector<Frame*>& vpFrames){
    std::cout << "[Initializer::VisualOnlyInitS1] Start" << std::endl;

    std::vector<Eigen::Vector3d> vPts3D;
    std::vector<Eigen::Vector2d> vPts2D1, vPts2D2;
    std::vector<unsigned long> vChainIds;

    std::vector<cv::Vec2f> cvPts2D1, cvPts2D2;
    int matchFId = 1;
    int bGood = false;
    for(; matchFId < vpFrames.size(); matchFId++){
        vPts2D1.clear();
        vPts2D2.clear();
        vChainIds.clear();
        cvPts2D1.clear();
        cvPts2D2.clear();

        int matchNum = mpFM->GetMatches(0, matchFId, vPts2D1, vPts2D2, vChainIds);
        std::cout << "[Initializer::VisualOnlyInitS1] matchNum=" << matchNum << std::endl;
        if(matchNum < 20){
            continue;
        }

        cvPts2D1.resize(matchNum); 
        cvPts2D2.resize(matchNum);
        std::vector<float> parallaxs;
        for(int i = 0; i < matchNum; i++){
            Eigen::Vector2d chainOffset = vPts2D2[i] - vPts2D1[i];
            float parallax = std::sqrt(chainOffset.dot(chainOffset));
            parallaxs.push_back(parallax);
            cvPts2D1[i] = cv::Vec2f(vPts2D1[i][0], vPts2D1[i][1]);
            cvPts2D2[i] = cv::Vec2f(vPts2D2[i][0], vPts2D2[i][1]);
        }
        std::sort(parallaxs.begin(), parallaxs.end());
        if(parallaxs[matchNum / 2] >= 18){
            bGood = true;
            std::cout << "[Initializer::VisualOnlyInitS1] Find good match id: " << matchFId << std::endl;
            break;
        }
    }
    if(!bGood){
        std::cout << "[Initializer::VisualOnlyInitS1] Cannot find good match. Try to marginalize the first frame and retry" << std::endl;
        return false;
    }

    cv::Mat mask, cvR21, cvt21, K;
    cv::eigen2cv(mK, K);
    cv::Mat EMat = cv::findEssentialMat(cvPts2D1, cvPts2D2, K, cv::RANSAC, 0.999, 3.84, mask);
    int inlier_cnt = cv::recoverPose(EMat, cvPts2D1, cvPts2D2, K, cvR21, cvt21, mask);
    Eigen::Matrix3d R21;
    Eigen::Vector3d t21;
    cv::cv2eigen(cvR21, R21);
    cv::cv2eigen(cvt21, t21);
    vpFrames.back()->SetTcw(R21, t21);

    vPts3D = TriangulateTwoFrame(vpFrames.front()->mRcw, vpFrames.front()->mtcw, 
        vpFrames.back()->mRcw, vpFrames.back()->mtcw, vPts2D1, vPts2D2, vChainIds);

    mInitIdx = matchFId;

    Optimizer::VisualOnlyInitBA(vpFrames.front(), vpFrames[matchFId], mpFM, vPts3D, vPts2D1, vPts2D2, vChainIds, mK);
    std::cout << "[Initializer::VisualOnlyInitS1] Done succeed" << std::endl;

    // norm frame pose and 3D point
    NormalizePoseAndPoint(vpFrames.front(), vpFrames[matchFId], vPts3D, vChainIds);

    return true;
}
    
bool Initializer::VisualOnlyInitS2(std::vector<Frame*>& vpFrames){
    std::cout << "[Initializer::VisualOnlyInitS2] Start" << std::endl;
    // first deal with frames between 0 and mInitIdx
    for(int i = 1; i < mInitIdx; i++){

        std::vector<Eigen::Vector3d> vPts3D;
        std::vector<Eigen::Vector2d> vPts2D, vPts1, vPts2;
        std::vector<unsigned long> vChainIds;
        int matchNum = mpFM->GetMatches(0, i, vPts3D, vPts2D, vPts1, vPts2, vChainIds);
        if(vPts3D.size() < 10)
            return false;

        Eigen::Matrix3d Rcw2 = vpFrames[i-1]->GetTcw().rotationMatrix();
        Eigen::Vector3d tcw2 = vpFrames[i-1]->GetTcw().translation();
        GeometryFunc::SolvePnP(vPts3D, vPts2D, mK, Rcw2, tcw2);
        vpFrames[i]->SetTcw(Rcw2, tcw2);

        TriangulateTwoFrame(vpFrames[0]->mRcw, vpFrames[0]->mtcw, 
            vpFrames[i]->mRcw, vpFrames[i]->mtcw, vPts1, vPts2, vChainIds);
    }
    std::cout << "[Initializer::VisualOnlyInitS2] Triangle between 0 and mInitIdx done" << std::endl;

    // second deal with frames between mInitIdx and mWindowSize - 1
    for(int i = mInitIdx + 1; i < vpFrames.size(); i++){
        std::vector<Eigen::Vector3d> vPts3D;
        std::vector<Eigen::Vector2d> vPts2D, vPts1, vPts2;
        std::vector<unsigned long> vChainIds;
        int matchNum = mpFM->GetMatches(i - 1, i, vPts3D, vPts2D, vPts1, vPts2, vChainIds);
        if(vPts3D.size() < 10)
            return false;

        Eigen::Matrix3d Rcw2 = vpFrames[i-1]->GetTcw().rotationMatrix();
        Eigen::Vector3d tcw2 = vpFrames[i-1]->GetTcw().translation();
        GeometryFunc::SolvePnP(vPts3D, vPts2D, mK, Rcw2, tcw2);
        vpFrames[i]->SetTcw(Rcw2, tcw2);

        TriangulateTwoFrame(vpFrames[i - 1]->mRcw, vpFrames[i - 1]->mtcw, 
            vpFrames[i]->mRcw, vpFrames[i]->mtcw, vPts1, vPts2, vChainIds);
    }

    std::cout << "[Initializer::VisualOnlyInitS2] Triangle between mInitIdx and last done" << std::endl;

    // third: deal with all other chains from 1 to mInitIdx
    for(int i = 2; i <= mInitIdx; i++){
        std::vector<Eigen::Vector3d> vPts3D;
        std::vector<Eigen::Vector2d> vPts2D, vPts1, vPts2;
        std::vector<unsigned long> vChainIds;
        int matchNum = mpFM->GetMatches(i - 1, i, vPts3D, vPts2D, vPts1, vPts2, vChainIds);

        TriangulateTwoFrame(vpFrames[i - 1]->mRcw, vpFrames[i - 1]->mtcw, 
            vpFrames[i]->mRcw, vpFrames[i]->mtcw, vPts1, vPts2, vChainIds);
    }
    std::cout << "[Initializer::VisualOnlyInitS2] Triangle left done" << std::endl;

    int goodChainNum = Optimizer::VisualOnlyBA(vpFrames, mpFM, mK);
    std::cout << "[Initializer::VisualOnlyInitS2] Done succeed" << std::endl;
    return true;
}

std::vector<Eigen::Vector3d> Initializer::TriangulateTwoFrame(const Eigen::Matrix3d& Rcw1, const Eigen::Vector3d& tcw1,
        const Eigen::Matrix3d& Rcw2, const Eigen::Vector3d& tcw2,
        const std::vector<Eigen::Vector2d>& vPts2D1, const std::vector<Eigen::Vector2d>& vPts2D2,
        const std::vector<unsigned long>& vChainIds){

    std::cout << "[Initializer::TriangulateTwoFrame] Start" << std::endl;
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

    std::cout << "[Initializer::TriangulateTwoFrame] Done" << std::endl;
    return vPts3D;
}

void Initializer::NormalizePoseAndPoint(Frame* pF1, Frame* pF2, std::vector<Eigen::Vector3d>& vPts3D,
                                        const std::vector<unsigned long>& vChainIds){
    std::cout << "[Initializer::NormalizePoseAndPoint] Start" << std::endl;
    std::vector<double> vZs;
    std::vector<int> vGoodIds;
    for(int i = 0; i < vPts3D.size(); i++){
        if(mpFM->IsChainPosSet(vChainIds[i])){
            vZs.emplace_back(vPts3D[i][2]);
            vGoodIds.emplace_back(i);
        }
    }
    sort(vZs.begin(), vZs.end());
    double medianDepth = vZs[(vZs.size() - 1) / 2];
    double invMedianDepth = 4.0 / medianDepth;

    // Norm pose
    auto Tcw = pF2->GetTcw();
    std::cout << "******************Tcw before norm:******************" << std::endl << Tcw.translation().transpose() << std::endl;
    Tcw.translation() *= invMedianDepth;
    pF2->SetTcw(Tcw);
    std::cout << "******************Tcw after norm:******************" << std::endl << Tcw.translation().transpose() << std::endl;

    // norm 3D points
    for(int i = 0; i < vPts3D.size(); i++){
        if(mpFM->IsChainPosSet(vChainIds[i])){
            std::cout << "pt before norm: " << vPts3D[i].transpose();
            Eigen::Vector3d normedWorldPos = vPts3D[i] * invMedianDepth;
            std::cout << "    pt after norm: " << normedWorldPos.transpose() << std::endl;;
            mpFM->UpdateWorldPos(vChainIds[i], normedWorldPos);   
        }
    }
    std::cout << "[Initializer::NormalizePoseAndPoint] Done" << std::endl;
}

bool Initializer::VisualInertialInit(std::vector<Frame*>& vpFrames){
    std::cout << "[Initializer::VisualInertialInit] Start" << std::endl;
    // manage initial value
    Eigen::Vector3d dirG(0, 0, 0);
    auto firstTimestamp = vpFrames.front()->mdTimestamp;
    for(int i = 1; i < vpFrames.size(); i++){
        auto pPreF = vpFrames[i - 1];
        auto pCurF = vpFrames[i];
        Eigen::Matrix3d Rwbi = pPreF->mRwc * mTcb.rotationMatrix();
        dirG -= Rwbi * pCurF->mpPreintegrator->GetDeltaV();
        double dT = pCurF->mdTimestamp - pPreF->mdTimestamp;
        // Eigen::Vector3d vel = (pCurF->GetBodyPosition() - pPreF->GetBodyPosition()) / dT;
        Eigen::Vector3d vel(0, 0, 0);
        pPreF->SetVelocity(vel);
        std::cout << "[Initializer::VisualInertialInit] frame=" << i << "  Pose:" << std::endl;
        std::cout << pCurF->GetTcw().rotationMatrix() << std::endl << pCurF->GetTcw().translation().transpose() << std::endl;
        std::cout << "[Initializer::VisualInertialInit] vel = " << vel.transpose() << std::endl;
    }
    dirG.normalize();
    Eigen::Vector3d gI(0, 0, -1);
    Eigen::Vector3d v = gI.cross(dirG);
    v = v / v.norm();
    double ang = acos(gI.dot(dirG));
    Eigen::Vector3d vzg = ang * v;
    // Rwg = Sophus::SO3d::exp(vzg).matrix();
    Eigen::Matrix3d Rwg = LieAlg::Exp(vzg);

    Eigen::Vector3d AccBias(0, 0, 0);
    Eigen::Vector3d GyrBias(0, 0, 0);

    double scale = 1.0;

    Optimizer::VIInitOptimize(vpFrames, Rwg, GyrBias, AccBias, scale, 1e10, 1e2);
    if(scale < 0.1){
        std::cout << "[Initializer::VisualInertialInit] Done fail" << std::endl;
        return false;
    }
    std::cout << "[Initializer::VisualInertialInit] Done Good" << std::endl;
    return true;
}

void Initializer::VisualInertialAlign(const Eigen::Matrix3d& Rwg, const Eigen::Vector3d& gyrBias, 
                                      const Eigen::Vector3d& accBias, double scale){

}



} // namespace Naive_SLAM_ROS
