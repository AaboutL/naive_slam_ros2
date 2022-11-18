//
// Created by hanfuyong on 2022/10/17.
//

#include "StateEstimationNode.h"

namespace Naive_SLAM_ROS{

StateEstimationNode::StateEstimationNode(const std::string& strParamFile):
mtStateEstimation(std::bind(&StateEstimationNode::Run, this)),
Node("state_estimation_node"){
    mpEstimator = new StateEstimator(strParamFile);

    pc_subscriptor_ = this->create_subscription<sensor_msgs::msg::PointCloud>("/feature_tracking/point_cloud", 100, 
        std::bind(&StateEstimationNode::PointCloudCallback, this, std::placeholders::_1));

    imu_subscriptor_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu0", 100, 
        std::bind(&StateEstimationNode::IMUCallback, this, std::placeholders::_1));

    kf_pose_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("keyframe_pose", 1000);
    
}

void StateEstimationNode::PointCloudCallback(const sensor_msgs::msg::PointCloud::ConstSharedPtr& pc_msg){
    std::vector<unsigned long> vChainIds;
    std::vector<Eigen::Vector2d> vPtsUn, vPts, vPtUnOffsets;
    for(int i = 0; i < pc_msg->points.size(); i++){
        unsigned long chainId = pc_msg->channels[0].values[i];
        Eigen::Vector2d ptUn(pc_msg->points[i].x, pc_msg->points[i].y);
        Eigen::Vector2d pt(pc_msg->channels[1].values[i], pc_msg->channels[2].values[i]);
        Eigen::Vector2d ptUnOffset(pc_msg->channels[3].values[i], pc_msg->channels[4].values[i]);
        vChainIds.emplace_back(chainId);
        vPtsUn.emplace_back(ptUn);
        vPts.emplace_back(pt);
        vPtUnOffsets.emplace_back(ptUnOffset);
    }

    auto ts = rclcpp::Time(pc_msg->header.stamp.sec, pc_msg->header.stamp.nanosec);
    // PointCloud pc(ts.seconds(), vChainIds, vPtsUn, vPts, vPtUnOffsets);
    PointCloud pc(ts.seconds(), std::to_string(pc_msg->header.stamp.sec) + std::to_string(pc_msg->header.stamp.nanosec), 
                  vChainIds, vPtsUn, vPts, vPtUnOffsets);

    mMutexBuffer.lock();
    mqPointClouds.emplace(pc);
    mMutexBuffer.unlock();
    mConditionVar.notify_one();
}

void StateEstimationNode::IMUCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg){
    double ax = imu_msg->linear_acceleration.x;
    double ay = imu_msg->linear_acceleration.y;
    double az = imu_msg->linear_acceleration.z;
    double wx = imu_msg->angular_velocity.x;
    double wy = imu_msg->angular_velocity.y;
    double wz = imu_msg->angular_velocity.z;

    auto ts = rclcpp::Time(imu_msg->header.stamp.sec, imu_msg->header.stamp.nanosec);
    // IMU imu(ts.seconds(), Eigen::Vector3d(ax, ay, az), Eigen::Vector3d(wx, wy, wz), -1);
    IMU imu(ts.seconds(), std::to_string(imu_msg->header.stamp.sec) + std::to_string(imu_msg->header.stamp.nanosec), 
            Eigen::Vector3d(ax, ay, az), Eigen::Vector3d(wx, wy, wz), -1);
    
    mMutexBuffer.lock();
    mqIMUs.emplace(imu);
    mMutexBuffer.unlock();
    mConditionVar.notify_one();
}

std::vector<std::pair<PointCloud, std::vector<IMU>>> StateEstimationNode::BindImageAndIMUs(){
    std::vector<std::pair<PointCloud, std::vector<IMU>>> mvMeasurements;
    while(true){
        if(mqIMUs.empty() || mqPointClouds.empty()){
            // until mqIMUs or mqPointClouds was all taken off, then return
            return mvMeasurements;
        }
        if(mqIMUs.back().mdTimestamp <= mqPointClouds.front().mdTimestamp){
            // The newest imu in mqIMUs is older than the oldest frame in mqPointClouds, just wait for more imu
            return mvMeasurements;
        }
        if(mqIMUs.front().mdTimestamp >= mqPointClouds.front().mdTimestamp){
            // The oldest imu came after the oldest frame, so the frames that came before imu need to be removed.
            mqPointClouds.pop();
            continue;
        }

        // Now we get some imus came before the oldest frame and some imus came after the oldest frame
        PointCloud onePointCloud = mqPointClouds.front();
        mqPointClouds.pop();
        std::vector<IMU> vIMUs;
        while(mqIMUs.front().mdTimestamp < onePointCloud.mdTimestamp){
            vIMUs.emplace_back(mqIMUs.front());
            mqIMUs.pop();
        }
        vIMUs.emplace_back(mqIMUs.front());
        mvMeasurements.emplace_back(std::make_pair(onePointCloud, vIMUs));
    }
    return mvMeasurements;
}

void StateEstimationNode::Run(){
    int i= 0;
    while(true){
        std::vector<std::pair<PointCloud, std::vector<IMU>>> vMeasurements;
        std::unique_lock<std::mutex> ulk(mMutexBuffer);
        mConditionVar.wait(ulk, [&]
        {
            return (vMeasurements = BindImageAndIMUs()).size() != 0;
        });
        ulk.unlock();
        for(auto & meas : vMeasurements){
            std::cout << "******************************************************* current frame id=" << i <<
                         " ********************************************************" << std::endl;
            i++;
            // for(int i = 0; i < meas.second.size(); i++){
            //     std::cout << meas.first.msTimestamp << "  " << meas.second[i].msTimestamp << std::endl;
            // }
            // std::cout << std::endl;
            if (i < 2) continue;
            std::cout  << "vMeas size = " << vMeasurements.size() << std::endl;
            mpEstimator->Estimate(meas);
            std::cout << "*****************************[StateEstimationNode::Run] mpEstimator->Estimate() done***************************************" << std::endl;
        }
    }
}


} // namespace naive_slam_ros