//
// Created by hanfuyong on 2022/10/27
//

#include "Initializer.h"
#include "GeometryFunc.h"

namespace Naive_SLAM_ROS{

Initializer::Initializer(int matchNumTh, float parallaxTh, const Eigen::Matrix3d& K, FeatureManager* pFM,
                        Sophus::SE3d& Tbc):
mMatchNumTh(matchNumTh), mParallaxTh(parallaxTh), mK(K), mInitIdx(0), mpFM(pFM), 
mTbc(Tbc), mTcb(Tbc.inverse()), mG(Eigen::Vector3d(0, 0, 9.81)){
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
    // int matchFId = 1;
    int matchFId = vpFrames.size() - 1;
    int bGood = false;
    // for(; matchFId < vpFrames.size(); matchFId++){
    for(; matchFId > 0; matchFId--){
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
        std::cout << "[Initializer::VisualOnlyInitS1] medium parallax=" << parallaxs[matchNum / 2] << std::endl;
        if(parallaxs[matchNum / 2] >= 30){
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
    // vpFrames.back()->SetTcw(R21, t21);
    vpFrames[matchFId]->SetTcw(R21, t21);

    auto vPts3DNew = GeometryFunc::TriangulateTwoFrame(vpFrames.front()->mRcw, vpFrames.front()->mtcw, 
        // vpFrames.back()->mRcw, vpFrames.back()->mtcw, mK, vPts2D1, vPts2D2, vChainIds);
        vpFrames[matchFId]->mRcw, vpFrames[matchFId]->mtcw, mK, vPts2D1, vPts2D2, vChainIds);
    for(int i = 0; i < vChainIds.size(); i++){
        if(vPts3DNew[i] != Eigen::Vector3d(0, 0, 0)){
            mpFM->SetWorldPos(vChainIds[i], vPts3DNew[i]);
        }
    }

    mInitIdx = matchFId;

    Optimizer::VisualOnlyInitBA(vpFrames.front(), vpFrames[matchFId], mpFM, vPts3DNew, vPts2D1, vPts2D2, vChainIds, mK);
    std::cout << "[Initializer::VisualOnlyInitS1] Done succeed" << std::endl;

    // norm frame pose and 3D point
    // NormalizePoseAndPoint(vpFrames.front(), vpFrames[matchFId], vPts3DNew, vChainIds);

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

        auto vPts3DNew = GeometryFunc::TriangulateTwoFrame(vpFrames[0]->mRcw, vpFrames[0]->mtcw, 
            vpFrames[i]->mRcw, vpFrames[i]->mtcw, mK, vPts1, vPts2, vChainIds);
        for(int i = 0; i < vChainIds.size(); i++){
            if(vPts3DNew[i] != Eigen::Vector3d(0, 0, 0)){
                mpFM->SetWorldPos(vChainIds[i], vPts3DNew[i]);
            }
        }
    }
    std::cout << "[Initializer::VisualOnlyInitS2] Triangle between 0 and mInitIdx done" << std::endl;

    // second deal with frames between mInitIdx and mWindowSize - 1
    for(int i = mInitIdx + 1; i < vpFrames.size(); i++){
        std::vector<Eigen::Vector3d> vPts3D;
        std::vector<Eigen::Vector2d> vPts2D, vPts1, vPts2;
        std::vector<unsigned long> vChainIds;
        int matchNum = mpFM->GetMatches(i - 1, i, vPts3D, vPts2D, vPts1, vPts2, vChainIds);
        if(vPts3D.size() < 10){
            std::cout << "[Initializer::VisualOnlyInitS2] vPts3D num between " << i - 1 << " and " << i << " are not enough! Init failed" << std::endl;
            return false;
        }

        Eigen::Matrix3d Rcw2 = vpFrames[i-1]->GetTcw().rotationMatrix();
        Eigen::Vector3d tcw2 = vpFrames[i-1]->GetTcw().translation();
        GeometryFunc::SolvePnP(vPts3D, vPts2D, mK, Rcw2, tcw2);
        vpFrames[i]->SetTcw(Rcw2, tcw2);

        auto vPts3DNew = GeometryFunc::TriangulateTwoFrame(vpFrames[i - 1]->mRcw, vpFrames[i - 1]->mtcw, 
            vpFrames[i]->mRcw, vpFrames[i]->mtcw, mK, vPts1, vPts2, vChainIds);
        for(int i = 0; i < vChainIds.size(); i++){
            if(vPts3DNew[i] != Eigen::Vector3d(0, 0, 0)){
                mpFM->SetWorldPos(vChainIds[i], vPts3DNew[i]);
            }
        }
    }

    std::cout << "[Initializer::VisualOnlyInitS2] Triangle between mInitIdx and last done" << std::endl;

    // third: deal with all other chains from 1 to mInitIdx
    for(int i = 2; i <= mInitIdx; i++){
        std::vector<Eigen::Vector3d> vPts3D;
        std::vector<Eigen::Vector2d> vPts2D, vPts1, vPts2;
        std::vector<unsigned long> vChainIds;
        int matchNum = mpFM->GetMatches(i - 1, i, vPts3D, vPts2D, vPts1, vPts2, vChainIds);

        auto vPts3DNew = GeometryFunc::TriangulateTwoFrame(vpFrames[i - 1]->mRcw, vpFrames[i - 1]->mtcw, 
            vpFrames[i]->mRcw, vpFrames[i]->mtcw, mK, vPts1, vPts2, vChainIds);
        for(int i = 0; i < vChainIds.size(); i++){
            if(vPts3DNew[i] != Eigen::Vector3d(0, 0, 0)){
                mpFM->SetWorldPos(vChainIds[i], vPts3DNew[i]);
            }
        }
    }
    std::cout << "[Initializer::VisualOnlyInitS2] Triangle left done" << std::endl;

    int goodChainNum = Optimizer::VisualOnlyBA(vpFrames, mpFM, mK);
    std::cout << "[Initializer::VisualOnlyInitS2] Done succeed" << std::endl;
    return true;
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
            Eigen::Vector3d normedWorldPos = vPts3D[i] * invMedianDepth;
            mpFM->UpdateWorldPos(vChainIds[i], normedWorldPos);   
            std::cout << "normed pt3D=" << normedWorldPos.transpose() << std::endl;
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
        Eigen::Vector3d vel(0, 0, 0); // since vel can init to zeros, so we can use all frames in window to initialize.
        pPreF->SetVelocity(vel);
        // std::cout << "[Initializer::VisualInertialInit] frame=" << i << "  Pose:" << std::endl;
        // std::cout << pCurF->GetTcw().rotationMatrix() << std::endl << pCurF->GetTcw().translation().transpose() << std::endl;
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

    Optimizer::InertialOnlyBA(vpFrames, Rwg, GyrBias, AccBias, scale, 1e10, 1e2);
    if(scale < 0.1){
        std::cout << "[Initializer::VisualInertialInit] Done fail" << std::endl;
        return false;
    }
    VisualInertialAlign(vpFrames, Rwg, GyrBias, AccBias, scale);

    Optimizer::VisualInertialInitBA(vpFrames, mpFM, mK, GyrBias, AccBias, 1e5, 1e2);

    std::cout << "[Initializer::VisualInertialInit] Done Good" << std::endl;
    return true;
}

void Initializer::VisualInertialAlign(std::vector<Frame*>& vpFrames, const Eigen::Matrix3d& Rwg, const Eigen::Vector3d& gyrBias, 
                                      const Eigen::Vector3d& accBias, double scale){
    Sophus::SE3d Tgw(Rwg.transpose(), Eigen::Vector3d::Zero());
    Eigen::Matrix3d Rgw = Tgw.rotationMatrix();
    Eigen::Vector3d tgw = Tgw.translation();
    for(int i = 0; i < vpFrames.size(); i++){ //  since vel can init to zeros, so we can use all frames in window to initialize.
        auto* pF = vpFrames[i];
        auto Twc = pF->GetTcw().inverse();
        Twc.translation() *= scale;
        Sophus::SE3d Tgc = Tgw * Twc;
        pF->SetTcw(Tgc.inverse());

        auto vel = pF->GetVelocity();
        pF->SetVelocity(Rgw * vel * scale);
        pF->mpPreintegrator->ReIntegrate(gyrBias, accBias);
        std::cout << "[Initializer::VisualInertialAlign] vel=" << pF->GetVelocity().transpose() << std::endl;
        std::cout << "[Initializer::VisualInertialAlign] pos=" << pF->GetTcw().translation().transpose() << std::endl;
    }

    mpFM->UpdateWorldPos(Rgw, tgw, scale);
}

void Initializer::SolveGyrBias(std::vector<Frame *> &vpFrames)
{
    Eigen::Matrix3d A;
    Eigen::Vector3d b, deltaGB;
    A.setZero();
    b.setZero();
    cv::FileStorage fs("/media/psf/ShareParallels/deltaR.yaml", cv::FileStorage::READ);
    // cv::FileStorage fs("/media/psf/ShareParallels/deltaJRg.yaml", cv::FileStorage::WRITE);
    cv::Mat mRs = fs["R1"].mat();
    std::vector<Eigen::Matrix3d> Rs;
    for(int i = 0; i < 10; i++){
        Eigen::Matrix3d R;
        cv::cv2eigen(mRs.rowRange(i * 3, (i+1)* 3), R);
        Rs.emplace_back(R);
    }
    cv::Mat JRgs = cv::Mat::zeros(30, 3, CV_32F);
    for(int i = 0; i < vpFrames.size()-1; i++){
        auto* pFi = vpFrames[i];
        auto* pFj = vpFrames[i+1];
        Eigen::Matrix3d tmp_A;
        tmp_A.setZero();
        Eigen::VectorXd tmp_b;
        tmp_b.setZero();
        Eigen::Matrix3d Rwbi = pFi->GetTwc().rotationMatrix() * pFi->GetTbc().rotationMatrix().transpose();
        Eigen::Matrix3d Rwbj = pFj->GetTwc().rotationMatrix() * pFj->GetTbc().rotationMatrix().transpose();
        Eigen::Matrix3d Rij = Rwbi.transpose() * Rwbj;
        // Eigen::Matrix3d Rij = Rs[i];
        std::cout << "[Initializer::SolveGyrBias] Rij: " << std::endl << Rij << std::endl;
        std::cout << "[Initializer::SolveGyrBias] ypr: " << GeometryFunc::R2ypr(Rij).transpose() << std::endl;
        tmp_A = pFj->mpPreintegrator->GetJRbg();
        // cv::Mat tmp;
        // cv::eigen2cv(tmp_A, tmp);
        // tmp.copyTo(JRgs.rowRange(i * 3, (i+1)* 3));
        // tmp_b = Sophus::SO3d(pFj->mpPreintegrator->GetDeltaR().transpose() * Rij).log();
        tmp_b = LieAlg::Log(pFj->mpPreintegrator->GetDeltaR().transpose() * Rij);

        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    // fs << "JRgs" << JRgs;
    // fs.release();
    deltaGB = A.ldlt().solve(b);
    std::cout << "[Initializer::SolveGyrBias] deltaGB=" << deltaGB.transpose() << std::endl;
    for(int i = 0; i < vpFrames.size(); i++){
        vpFrames[i]->mpPreintegrator->ReIntegrate(deltaGB, Eigen::Vector3d::Zero());
    }
}

bool Initializer::LinearAlignment(std::vector<Frame *> &vpFrames, Eigen::Vector3d& g, Eigen::VectorXd &x)
{
    int stateDim = vpFrames.size() * 3 + 3 + 1;
    Eigen::MatrixXd A{stateDim, stateDim};
    A.setZero();
    Eigen::VectorXd b{stateDim};
    b.setZero();
    for(int i = 0; i < vpFrames.size() - 1; i++){
        auto* pFi = vpFrames[i];
        auto* pFj = vpFrames[i + 1];
        Sophus::SE3d Tbiw = mTbc * pFi->GetTcw();
        Sophus::SE3d Tbjw = mTbc * pFj->GetTcw();
        Eigen::MatrixXd tmp_A(6, 10);
        tmp_A.setZero();
        Eigen::VectorXd tmp_b(6);
        tmp_b.setZero();
        double dt = pFj->mpPreintegrator->GetDeltaT();
        tmp_A.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = 0.5 * Tbiw.rotationMatrix() * dt * dt;
        tmp_A.block<3, 1>(0, 9) = Tbiw.rotationMatrix() * (pFj->GetTwc().translation() - pFi->GetTwc().translation()) / 100.0;
        tmp_b.block<3, 1>(0, 0) = pFj->mpPreintegrator->GetDeltaP() + Tbiw.rotationMatrix() * Tbjw.rotationMatrix().transpose()*mTbc.translation() - mTbc.translation();

        tmp_A.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = Tbiw.rotationMatrix() * Tbjw.rotationMatrix().transpose();
        tmp_A.block<3, 3>(3, 6) = Tbiw.rotationMatrix() * dt;
        tmp_b.block<3, 1>(3, 0) = pFj->mpPreintegrator->GetDeltaV();

        Eigen::MatrixXd r_A = tmp_A.transpose() * tmp_A;
        Eigen::VectorXd r_b = tmp_A.transpose() * tmp_b;
        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
        b.segment<6>(i * 3) += r_b.head<6>();
        A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
        b.tail<4>() += r_b.tail<4>();
        A.block<6, 4>(i * 3, stateDim - 4) += r_A.topRightCorner<6, 4>();
        A.block<4, 6>(stateDim - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
    }
    A = A * 1000;
    b = b * 1000;
    x = A.ldlt().solve(b);

    double s = x(stateDim - 1) / 100.0;
    std::cout << "[Initializer::LinearAlignment] scale="<< s << std::endl;
    g = x.segment<3>(stateDim - 4);
    std::cout << "[Initializer::LinearAlignment] g="<< g.transpose() << "  norm=" << g.norm() << std::endl;
    if(fabs(g.norm() - mG.norm()) > 1.0 || s < 0)
    {
        return false;
    }

    RefineGravity(vpFrames, g, x);
    s = x.tail<1>()(0) / 100.0;
    x.tail<1>()(0) = s;
    std::cout << "[Initializer::LinearAlignment] after refine scale="<< s << std::endl;
    std::cout << "[Initializer::LinearAlignment] after refine g="<< g.transpose() << "  norm=" << g.norm() << std::endl;
    if(s < 0.0)
        return false;
    return true;
}

Eigen::MatrixXd Initializer::TangentBasis(const Eigen::Vector3d &g0){
    Eigen::Vector3d b, c;
    Eigen::Vector3d a = g0.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    Eigen::MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}

void Initializer::RefineGravity(std::vector<Frame *> &vpFrames, Eigen::Vector3d& g, Eigen::VectorXd &x)
{
    Eigen::Vector3d g0 = g.normalized() * mG.norm(); // direction of g, magnitute of mG
    Eigen::Vector3d lx, ly;
    int stateDim = vpFrames.size() * 3 + 2 + 1;
    Eigen::MatrixXd A{stateDim, stateDim};
    A.setZero();
    Eigen::VectorXd b{stateDim};
    b.setZero();

    for (int k = 0; k < 4; k++){
        Eigen::MatrixXd lxly(3, 2);
        lxly = TangentBasis(g0);

        for(int i = 0; i < vpFrames.size() - 1; i++){
            auto* pFi = vpFrames[i];
            auto* pFj = vpFrames[i + 1];
            Sophus::SE3d Tbiw = mTbc * pFi->GetTcw();
            Sophus::SE3d Tbjw = mTbc * pFj->GetTcw();
            Eigen::MatrixXd tmp_A(6, 9);
            tmp_A.setZero();
            Eigen::VectorXd tmp_b(6);
            tmp_b.setZero();
            double dt = pFj->mpPreintegrator->GetDeltaT();
            tmp_A.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
            tmp_A.block<3, 2>(0, 6) = 0.5 * Tbiw.rotationMatrix() * dt * dt * lxly;
            tmp_A.block<3, 1>(0, 8) = Tbiw.rotationMatrix() * (pFj->GetTwc().translation() - pFi->GetTwc().translation()) / 100.0;
            tmp_b.block<3, 1>(0, 0) = pFj->mpPreintegrator->GetDeltaP() + Tbiw.rotationMatrix() * Tbjw.rotationMatrix().transpose()*mTbc.translation() - mTbc.translation()
                                      - Tbiw.rotationMatrix() * dt * dt * 0.5 * g0;

            tmp_A.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
            tmp_A.block<3, 3>(3, 3) = Tbiw.rotationMatrix() * Tbjw.rotationMatrix().transpose();
            tmp_A.block<3, 2>(3, 6) = Tbiw.rotationMatrix() * dt * lxly;
            tmp_b.block<3, 1>(3, 0) = pFj->mpPreintegrator->GetDeltaV() - Tbiw.rotationMatrix() * dt * g0;

            Eigen::MatrixXd r_A = tmp_A.transpose() * tmp_A;
            Eigen::VectorXd r_b = tmp_A.transpose() * tmp_b;
            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
            b.segment<6>(i * 3) += r_b.head<6>();
            A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
            b.tail<4>() += r_b.tail<4>();
            A.block<6, 3>(i * 3, stateDim - 3) += r_A.topRightCorner<6, 3>();
            A.block<3, 6>(stateDim - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
        }
        A = A * 1000.0;
        b = b * 1000.0;
        x = A.ldlt().solve(b);
        Eigen::VectorXd dg = x.segment<2>(stateDim - 3);
        g0 = (g0 + lxly * dg).normalized() * mG.norm();
    }
    g = g0;
}

bool Initializer::VIAlign(std::vector<Frame *> &vpFrames){
    SolveGyrBias(vpFrames);
    Eigen::Vector3d g; 
    Eigen::VectorXd x;
    bool flag = LinearAlignment(vpFrames, g, x);
    if(!flag)
        return false;

    Eigen::Vector3d mGN = mG.normalized();
    Eigen::Vector3d gN = g.normalized();
    Eigen::Vector3d v = mGN.normalized().cross(gN.normalized());
    v = v / v.norm();
    double ang = acos(mGN.dot(gN));
    Eigen::Vector3d vzg = ang * v;
    // Rwg = Sophus::SO3d::exp(vzg).matrix();
    Eigen::Matrix3d Rwg = LieAlg::Exp(vzg);
    VisualInertialAlign(vpFrames, Rwg, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), x.tail<1>()(0));

    Eigen::Vector3d gyrBias = vpFrames.front()->mpPreintegrator->GetGyrBias();
    Eigen::Vector3d accBias;
    accBias.setZero();
    Optimizer::VisualInertialInitBA(vpFrames, mpFM, mK, gyrBias, accBias, 1e5, 1e2);
    return true;
}

} // namespace Naive_SLAM_ROS
