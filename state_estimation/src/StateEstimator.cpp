//
// Created by hanfuyong on 2022/10/20
//

#include "StateEstimator.h"

namespace Naive_SLAM_ROS{

StateEstimator::StateEstimator(const std::string& strParamFile):
mFrameId(0), mState(WAIT){
    cv::FileStorage fs(strParamFile.c_str(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "[Estimator] Param file not exist..." << std::endl;
        exit(0);
    }
    mWindowSize = fs["WindowSize"];
    mK = Eigen::Matrix3d::Identity();
    mK(0, 0) = fs["Camera.fx"];
    mK(1, 1) = fs["Camera.fy"];
    mK(0, 2) = fs["Camera.cx"];
    mK(1, 2) = fs["Camera.cy"];
    // mpFM = std::make_shared<FeatureManager>(mWindowSize);
    mpFM = new FeatureManager(mWindowSize);
    mLastGyrBias.setZero();
    mLastAccBias.setZero();
    mGyrNoise = fs["IMU.GyrNoise"];
    mAccNoise = fs["IMU.AccNoise"];
    mGyrBiasWalk = fs["IMU.GyrBiasWalk"];
    mAccBiasWalk = fs["IMU.AccBiasWalk"];
    mIMUFrequency = fs["IMU.Frequency"];
    cv::Mat cvTbc = fs["IMU.Tbc"].mat();
    Eigen::Matrix4d eTbc;
    cv::cv2eigen(cvTbc, eTbc);
    mTbc = Sophus::SE3d(eTbc);

    mpInitializer = new Initializer(20, 18, mK, mpFM, mTbc);
    mpLastFrame = nullptr;
    mvLastIMUs = std::vector<IMU>();
}

void StateEstimator::Estimate(const std::pair<PointCloud, std::vector<IMU>>& meas){
    auto pc = meas.first;
    auto imus = meas.second;
    mpFM->Manage(pc, mFrameId, mvpFrames.size());

    Frame* pframe = new Frame(pc, mTbc);
    mvpFrames.emplace_back(pframe);

    mFrameId++;

    Preintegrate(pframe, imus);
    if(mvpFrames.size() <= 2){// first frame
        mpLastFrame = pframe;
        mvLastIMUs = imus;
        return;
    } 
    else{
        // +1 is for IMU initialization. once init is done, then Marginalize one frame(front or back)
        if(mState == WAIT){
            // int flag = IsInitKeyframe();
            int flag = 1;
            if(flag == -1){
                Reset();
                return;
            }
            else if(flag == 0){
                mMargFlag = MARG_SECOND_LASTEST;
            }
            else{
                mMargFlag = SLEEP;
            }
            Marginalize();
            if(mMargFlag == SLEEP && mvpFrames.size() == mWindowSize + 1){
                mState = INIT;
            }
        }
        if(mState == INIT){
            if(!CheckIMUObservability()){
                Reset();
                return;
            }
            int len2=0, len3 = 0, len4 = 0, len5=0;
            for(auto& [id, chain] : mpFM->GetChains()){
                int cl = chain.GetChainLen();
                if(cl == 2) len2++;
                if(cl == 3) len3++;
                if(cl == 4) len4++;
                if(cl >= 5) len5++;
            }
            std::cout << "chain num=" << mpFM->GetChains().size() << "  len2=" << len2
                      << "  len3=" << len3 << "  len4=" << len4 << "  len5=" << len5 << std::endl;

            bool flag = Initialize();
            if(flag){
                mState = ESTIMATE;
                mMargFlag == MARG_OLD;
                Marginalize();
                mpLastFrame = pframe;
                mvLastIMUs = imus;
                exit(0);
                return;
            }
            else{
                Reset();
                return;
            }
        }
        if(mState == ESTIMATE){
            SelectMarginalizePosition();
            SolveOdometry();
            Marginalize();
            mpLastFrame = pframe;
            mvLastIMUs = imus;
        }

    }
}

bool StateEstimator::Initialize(){
    bool bS1 = mpInitializer->VisualOnlyInitS1(mvpFrames);
    if(!bS1){
        return false;
    }

    bool bS2 = mpInitializer->VisualOnlyInitS2(mvpFrames);
    if(!bS2){
        return false;
    }
    // bool bVI = mpInitializer->VisualInertialInit(mvpFrames);
    bool bVI = mpInitializer->VIAlign(mvpFrames);
    if(!bVI){
        return false;
    } 
    return true;
}

void StateEstimator::Preintegrate(Frame* pFrame, const std::vector<IMU>& vIMUs){
    std::cout << "[StateEstimator::Preintegrate] Start" << std::endl;

    double sqrtFreq = sqrt(mIMUFrequency);
    double GyrN = mGyrNoise * sqrtFreq;
    double AccN = mAccNoise * sqrtFreq;
    double GyrBiasW = mGyrBiasWalk / sqrtFreq;
    double AccBiasW = mAccBiasWalk / sqrtFreq;
    Preintegrator *preintegrator = new Preintegrator(GyrN, AccN, 
        GyrBiasW, AccBiasW, mLastGyrBias, mLastAccBias);

    for(int i = 0; i < vIMUs.size()-1; i++){
        auto imu = vIMUs[i];
        std::cout << "imu: acc: " << imu.mAcc.transpose() << "  gyr: " << imu.mGyr.transpose() << std::endl;
        // if(i == 0){
        //     if(mvLastIMUs.empty())
        //         continue;
        //     auto lastImu = *(mvLastIMUs.end() - 2);
        //     double dt = imu.mdTimestamp - lastImu.mdTimestamp;
        //     double dtPre = mpLastFrame->mdTimestamp - lastImu.mdTimestamp;
        //     Eigen::Vector3d midAcc = (lastImu.mAcc + (imu.mAcc - lastImu.mAcc) / dt * dtPre + imu.mAcc) * 0.5;
        //     Eigen::Vector3d midGyr = (lastImu.mGyr + (imu.mGyr - lastImu.mGyr) / dt * dtPre + imu.mGyr) * 0.5;
        //     preintegrator->Integrate(IMU(-1, midAcc, midGyr, (dt - dtPre)));
        // }
        // else if(i > 0 && i < vIMUs.size() - 1){
        //     double dt = imu.mdTimestamp - vIMUs[i-1].mdTimestamp;
        //     Eigen::Vector3d midAcc = (vIMUs[i-1].mAcc + imu.mAcc) * 0.5;
        //     Eigen::Vector3d midGyr = (vIMUs[i-1].mGyr + imu.mGyr) * 0.5;
        //     preintegrator->Integrate(IMU(-1, midAcc, midGyr, dt));
        // }
        // else{
        //     double dtPre = pFrame->mdTimestamp - vIMUs[i - 1].mdTimestamp;
        //     double dt = imu.mdTimestamp - vIMUs[i-1].mdTimestamp;
        //     Eigen::Vector3d midAcc = (vIMUs[i-1].mAcc + vIMUs[i-1].mAcc + (imu.mAcc - vIMUs[i-1].mAcc) / dt * dtPre) * 0.5;
        //     Eigen::Vector3d midGyr = (vIMUs[i-1].mGyr + vIMUs[i-1].mGyr + (imu.mGyr - vIMUs[i-1].mGyr) / dt * dtPre) * 0.5;
        //     preintegrator->Integrate(IMU(-1, midAcc, midGyr, dtPre));
        // }
        if(i < vIMUs.size() -1){
            double dt = imu.mdTimestamp - vIMUs[i+1].mdTimestamp;
            Eigen::Vector3d midAcc = (vIMUs[i+1].mAcc + imu.mAcc) * 0.5;
            Eigen::Vector3d midGyr = (vIMUs[i+1].mGyr + imu.mGyr) * 0.5;
            preintegrator->Integrate(IMU(-1, midAcc, midGyr, dt));
        }
        else{
            double dt = imu.mdTimestamp - vIMUs[i+1].mdTimestamp;
            preintegrator->Integrate(IMU(-1, imu.mAcc, imu.mGyr, dt));
        }
    }
    std::cout << "dR: " << std::endl << preintegrator->GetDeltaR() << std::endl;
    std::cout << "JRg: " << std::endl << preintegrator->GetJRbg() << std::endl;
    pFrame->SetPreintegrator(preintegrator);
    std::cout << "[StateEstimator::Preintegrate] Done" << std::endl;
}

void StateEstimator::Marginalize(){
    std::cout << "[StateEstimator::Marginalize] mvpFrame size before marginalize = " << mvpFrames.size() << std::endl;
    // if((mState == INIT || mMargFlag == MARG_OLD) && mvpFrames.size() >= mWindowSize + 1){
    if(mMargFlag == MARG_OLD){
        std::cout << "[StateEstimator::Marginalize] marginalize old" << std::endl;
        auto* pFDel = mvpFrames.front();
        mvpFrames.erase(mvpFrames.begin());
        mpFM->EraseFront();
        delete pFDel;
        pFDel = nullptr;

        // triangle matches between mWindowSize - 2 and mWindowSize - 1
        std::vector<Eigen::Vector3d> vPts3D;
        std::vector<Eigen::Vector2d> vPts2D, vPts1, vPts2;
        std::vector<unsigned long> vChainIds;
        int matchNum = mpFM->GetMatches(mWindowSize - 2, mWindowSize - 1, vPts3D, vPts2D, vPts1, vPts2, vChainIds);
        auto vPts3DNew = GeometryFunc::TriangulateTwoFrame(mvpFrames[mWindowSize - 2]->mRcw, mvpFrames[mWindowSize - 2]->mtcw, 
            mvpFrames[mWindowSize - 1]->mRcw, mvpFrames[mWindowSize - 1]->mtcw, mK, vPts1, vPts2, vChainIds);
        for(int i = 0; i < vChainIds.size(); i++){
            if(vPts3DNew[i] != Eigen::Vector3d(0, 0, 0)){
                mpFM->SetWorldPos(vChainIds[i], vPts3DNew[i]);
            }
        }
    }
    // if(mMargFlag == MARG_SECOND_LASTEST && mvpFrames.size() >= mWindowSize + 1){
    if(mMargFlag == MARG_SECOND_LASTEST){
        std::cout << "[StateEstimator::Marginalize] marginalize second new" << std::endl;
        auto it = mvpFrames.end() - 2;
        auto* pFDel = *it;
        auto* pFNew = mvpFrames.back();

        for(auto& imu : pFNew->mpPreintegrator->GetIMUs()){
            pFDel->mpPreintegrator->Integrate(imu);
        }
        pFNew->mpPreintegrator = pFDel->mpPreintegrator;
        pFDel->mpPreintegrator = nullptr;

        mvpFrames.erase(it);
        mpFM->EraseBack(mvpFrames.size());
        delete pFDel;
        pFDel = nullptr;
    }
    std::cout << "[StateEstimator::Marginalize] mvpFrame size after marginalize = " << mvpFrames.size() << std::endl;
}

void StateEstimator::Reset(){
    std::cout << "**********************************************************************************************************************"<< std::endl;
    std::cout << "                                             [StateEstimator::Reset]                                                  "<< std::endl;
    std::cout << "**********************************************************************************************************************"<< std::endl;
    mFrameId = 0;
    mState = WAIT;
    mpFM->Reset();

    std::cout << "[StateEstimator::Reset] before mvpFrame size=" << mvpFrames.size() << std::endl;
    auto it = mvpFrames.begin();
    while(it != mvpFrames.end()){
        auto pF = *it;
        mvpFrames.erase(it);
        delete pF;
        pF = nullptr;
    }
    std::cout << "[StateEstimator::Reset] after mvpFrame size=" << mvpFrames.size() << std::endl;

    mLastGyrBias.setZero();
    mLastAccBias.setZero();

    mpInitializer->Reset();
    mpLastFrame = nullptr;
    mvLastIMUs = std::vector<IMU>();
}

bool StateEstimator::CheckIMUObservability(){
    std::cout << "[StateEstimator::CheckIMUObservability] Start" << std::endl;
    Eigen::Vector3d sum_acc(0, 0, 0);
    std::vector<Eigen::Vector3d> vAcc(mvpFrames.size());
    for(int i = 0; i < mvpFrames.size(); i++){
        double dt = mvpFrames[i]->mpPreintegrator->GetDeltaT();
        auto acc = mvpFrames[i]->mpPreintegrator->GetDeltaV() / dt;
        sum_acc += acc;
        vAcc[i] = acc;
    }

    Eigen::Vector3d avg_acc = sum_acc / (mvpFrames.size() - 1);
    double var = 0;
    for(int i = 0; i < mvpFrames.size(); i++){
        var += (vAcc[i] - avg_acc).dot(vAcc[i] - avg_acc);
    }
    var = sqrt(var / (mvpFrames.size() - 1));
    std::cout << "acc var = " << var << std::endl;
    if(var < 0.25){
        std::cout << "[StateEstimator::CheckIMUObservability] Done failed!" << std::endl;
        return false;
    }
    std::cout << "[StateEstimator::CheckIMUObservability] Done succeed!" << std::endl;
    return true;
}

void StateEstimator::SelectMarginalizePosition(){
    std::vector<Eigen::Vector3d> vPts3D;
    std::vector<Eigen::Vector2d> vPts2D1, vPts2D2;
    std::vector<unsigned long> vChainIds;

    std::vector<cv::Vec2f> cvPts2D1, cvPts2D2;

    // int matchNum = mpFM->GetMatches(mWindowSize - 1, mWindowSize, vPts2D1, vPts2D2, vChainIds);
    int matchNum = mpFM->GetMatches(mvpFrames.size() - 3, mvpFrames.size() - 2, vPts2D1, vPts2D2, vChainIds);
    if(matchNum < 20){
        mMargFlag = MARG_OLD;
        return;
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
    if(parallaxs[matchNum / 2] > 10){
        mMargFlag = MARG_OLD;
        return;
    }
    mMargFlag = MARG_SECOND_LASTEST;
}

int StateEstimator::IsInitKeyframe(){
    std::vector<Eigen::Vector3d> vPts3D;
    std::vector<Eigen::Vector2d> vPts2D1, vPts2D2;
    std::vector<unsigned long> vChainIds;
    std::vector<cv::Vec2f> cvPts2D1, cvPts2D2;

    int matchNum = mpFM->GetMatches(mvpFrames.size() - 3, mvpFrames.size() - 2, vPts2D1, vPts2D2, vChainIds);

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
    std::cout <<"[StateEstimator::IsInitKeyframe] match num = " << matchNum << "  parallax=" << parallaxs[matchNum / 2] << std::endl;
    // if(matchNum > 60 && parallaxs[matchNum / 2] > 10){
    if(matchNum < 40){
        return -1; // failed
    }
    if(parallaxs[matchNum / 2] > 10){
        return 1; // kf
    }
    return 0;
    // return 1;
}

bool StateEstimator::SolveOdometry(){
    std::vector<Eigen::Vector3d> vPts3D;
    std::vector<Eigen::Vector2d> vPts2D, vPts1, vPts2;
    std::vector<unsigned long> vChainIds;
    int matchNum = mpFM->GetMatches(mWindowSize - 1, mWindowSize, vPts3D, vPts2D, vPts1, vPts2, vChainIds);
    if(vPts3D.size() < 10)
        return false;

    Eigen::Matrix3d Rcw2 = mvpFrames[mWindowSize - 1]->GetTcw().rotationMatrix(); // as initial guess for pnp
    Eigen::Vector3d tcw2 = mvpFrames[mWindowSize - 1]->GetTcw().translation();
    GeometryFunc::SolvePnP(vPts3D, vPts2D, mK, Rcw2, tcw2);
    mvpFrames[mWindowSize]->SetTcw(Rcw2, tcw2); // as initial value for pose optimization
    mvpFrames[mWindowSize]->SetVelocity(Eigen::Vector3d(0, 0, 0));
    Optimizer::VisualInertialOptimize(mvpFrames, mpFM, mK, (mMargFlag==MARG_SECOND_LASTEST));
}

} // namespace Naive_SLAM_ROS
