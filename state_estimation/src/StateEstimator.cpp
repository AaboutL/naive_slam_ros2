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
    cv::Mat cvTbc = fs["IMU.Tbc"].mat();
    Eigen::Matrix4d eTbc;
    cv::cv2eigen(cvTbc, eTbc);
    mTbc = Sophus::SE3(eTbc);

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
    if(mvpFrames.size() == 1){// first frame
        mpLastFrame = pframe;
        mvLastIMUs = imus;
        return;
    } 
    else{
        // Preintegrate(pframe, imus);

        if(mState == INITS1 || mState == INITS2){
            int flag = VisualOnlyInit();
            if(flag == 1){
                mState = INITS2;
            }
            if(flag == 2){
                mState = VIINIT;
            }
        }
        if(mState == VIINIT){
            mpInitializer->VisualInertialInit(mvpFrames);
        }
        mpLastFrame = pframe;
        mvLastIMUs = imus;

        // TODO: marginalization
    }
}

int StateEstimator::VisualOnlyInit(){
    if(mState == INITS1){
        std::cout << "[StateEstimator::VisualOnlyInit] InitS1 Start" << std::endl;
        std::vector<Eigen::Vector2d> vPtsUn1, vPtsUn2;
        std::vector<unsigned long> vChainIds;
        std::vector<Eigen::Vector3d> vPts3D;
        bool bS1 = mpInitializer->VisualOnlyInitS1(mvpFrames, vPts3D, vPtsUn1, vPtsUn2, vChainIds);
        if(!bS1){
            std::cout << "[StateEstimator::VisualOnlyInit] InitS1 Done failed" << std::endl;
            return -1;
        }
        // Optimizer::VisualOnlyInitBA(mvpFrames.front(), mvpFrames.back(), mpFM, vPts3D, vPtsUn1, vPtsUn2, vChainIds, mK);
        std::cout << "[StateEstimator::VisualOnlyInit] InitS1 Done succeed" << std::endl;
        return 1;
    }

    // +1 is for IMU initialization. once init is done, then Marginalize one frame(front or back)
    if(mState == INITS2 && mvpFrames.size() == mWindowSize + 1){
        std::cout << "[StateEstimator::VisualOnlyInit] InitS2 Start" << std::endl;
        bool bS2 = mpInitializer->VisualOnlyInitS2(mvpFrames);
        if(!bS2){
            std::cout << "[StateEstimator::VisualOnlyInit] InitS2 Done failed" << std::endl;
            return -1;
        }
        // int goodChainNum = Optimizer::VisualOnlyBA(mvpFrames, mpFM, mK);
        std::cout << "[StateEstimator::VisualOnlyInit] InitS2 Done succeed" << std::endl;
        return 2;
    }
    return 0;
}

void StateEstimator::Preintegrate(Frame* pFrame, const std::vector<IMU>& vIMUs){
    std::cout << "[StateEstimator::Preintegrate] Start" << std::endl;
    Preintegrator *preintegrator = new Preintegrator(mGyrNoise, mAccNoise, 
        mGyrBiasWalk, mAccBiasWalk, mLastGyrBias, mLastAccBias);

    for(int i = 0; i < vIMUs.size(); i++){
        auto imu = vIMUs[i];
        if(i == 0){
            if(mvLastIMUs.empty())
                continue;
            auto lastImu = *(mvLastIMUs.end() - 2);
            double dt = imu.mdTimestamp - lastImu.mdTimestamp;
            double dtPre = mpLastFrame->mdTimestamp - lastImu.mdTimestamp;
            Eigen::Vector3d midAcc = (lastImu.mAcc + (imu.mAcc - lastImu.mAcc) / dt * dtPre + imu.mAcc) * 0.5;
            Eigen::Vector3d midGyr = (lastImu.mGyr + (imu.mGyr - lastImu.mGyr) / dt * dtPre + imu.mGyr) * 0.5;
            preintegrator->Integrate(IMU(-1, midAcc, midGyr, (dt - dtPre)));
        }
        else if(i != 0 && i < vIMUs.size() - 1){
            double dt = imu.mdTimestamp - vIMUs[i-1].mdTimestamp;
            Eigen::Vector3d midAcc = (vIMUs[i-1].mAcc + imu.mAcc) * 0.5;
            Eigen::Vector3d midGyr = (vIMUs[i-1].mGyr + imu.mGyr) * 0.5;
            preintegrator->Integrate(IMU(-1, midAcc, midGyr, dt));
        }
        else{
            double dtPre = pFrame->mdTimestamp - vIMUs[i - 1].mdTimestamp;
            double dt = imu.mdTimestamp - vIMUs[i-1].mdTimestamp;
            Eigen::Vector3d midAcc = (vIMUs[i-1].mAcc + vIMUs[i-1].mAcc + (imu.mAcc - vIMUs[i-1].mAcc) / dt * dtPre) * 0.5;
            Eigen::Vector3d midGyr = (vIMUs[i-1].mGyr + vIMUs[i-1].mGyr + (imu.mGyr - vIMUs[i-1].mGyr) / dt * dtPre) * 0.5;
            preintegrator->Integrate(IMU(-1, midAcc, midGyr, dtPre));
        }
    }
    pFrame->SetPreintegrator(preintegrator);
    std::cout << "[StateEstimator::Preintegrate] Done" << std::endl;
}

} // namespace Naive_SLAM_ROS
