//
// Created by hanfuyong on 2022/10/20
//

#include "StateEstimator.h"

namespace Naive_SLAM_ROS{

StateEstimator::StateEstimator(const std::string& strParamFile):
mFrameId(0), mState(INIT){
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
        if(mState == INIT && mvpFrames.size() == mWindowSize + 1){
            int flag = VisualOnlyInit();
            if(flag)
                mpInitializer->VisualInertialInit(mvpFrames);
        }
        if(mState == ESTIMATE){
            // TODO
        }
        mpLastFrame = pframe;
        mvLastIMUs = imus;

        // TODO: marginalization
        Marginalize();
    }
}

bool StateEstimator::VisualOnlyInit(){
    // +1 is for IMU initialization. once init is done, then Marginalize one frame(front or back)
    bool bS1 = mpInitializer->VisualOnlyInitS1(mvpFrames);
    if(!bS1){
        return false;
    }

    bool bS2 = mpInitializer->VisualOnlyInitS2(mvpFrames);
    if(!bS2){
        return false;
    }
    return true;
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

void StateEstimator::Marginalize(){
    std::cout << "mvpFrame size before marginalize = " << mvpFrames.size() << std::endl;
    if(mState == INIT && mvpFrames.size() == mWindowSize + 1){
        auto* pFDel = mvpFrames.front();
        mvpFrames.erase(mvpFrames.begin());
        std::cout << "test1" << std::endl;
        mpFM->EraseFront();
        std::cout << "test2" << std::endl;
        delete pFDel;
        std::cout << "test3" << std::endl;
        pFDel = nullptr;
        std::cout << "test4" << std::endl;
    }
    std::cout << "mvpFrame size after marginalize = " << mvpFrames.size() << std::endl;

}

} // namespace Naive_SLAM_ROS
