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
    if(mvpFrames.size() == 1){// first frame
        mpLastFrame = pframe;
        mvLastIMUs = imus;
        return;
    } 
    else{
        // +1 is for IMU initialization. once init is done, then Marginalize one frame(front or back)
        if(mState == INIT && mvpFrames.size() == mWindowSize + 1){
            if(!CheckIMUObservability()){
                Reset();
                return;
            }
            int flag = Initialize();
            if(flag){
                mState = ESTIMATE;
            }
            else{
                Reset();
                return;
            }
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

bool StateEstimator::Initialize(){
    bool bS1 = mpInitializer->VisualOnlyInitS1(mvpFrames);
    if(!bS1){
        return false;
    }

    bool bS2 = mpInitializer->VisualOnlyInitS2(mvpFrames);
    if(!bS2){
        return false;
    }
    bool bVI = mpInitializer->VisualInertialInit(mvpFrames);
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
        else if(i > 0 && i < vIMUs.size() - 1){
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

void StateEstimator::Marginalize(int margPos){
    std::cout << "mvpFrame size before marginalize = " << mvpFrames.size() << std::endl;
    if((mState == INIT || margPos == 0) && mvpFrames.size() == mWindowSize + 1){
        auto* pFDel = mvpFrames.front();
        mvpFrames.erase(mvpFrames.begin());
        mpFM->EraseFront();
        delete pFDel;
        pFDel = nullptr;
    }
    if(margPos == mWindowSize && mvpFrames.size() == mWindowSize + 1){
        auto* pFDel = mvpFrames.back();
        mvpFrames.pop_back();
        mpFM->EraseBack();
        delete pFDel;
        pFDel = nullptr;
    }
    std::cout << "mvpFrame size after marginalize = " << mvpFrames.size() << std::endl;

}

void StateEstimator::Reset(){
    std::cout << "**********************************************************************************************************************"<< std::endl;
    std::cout << "                                             [StateEstimator::Reset]                                                  "<< std::endl;
    std::cout << "**********************************************************************************************************************"<< std::endl;
    mFrameId = 0;
    mState = INIT;
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
    if(var < 0.25){
        std::cout << "[StateEstimator::CheckIMUObservability] Done failed!" << std::endl;
        return false;
    }
    std::cout << "[StateEstimator::CheckIMUObservability] Done succeed!" << std::endl;
    return true;
}

} // namespace Naive_SLAM_ROS
