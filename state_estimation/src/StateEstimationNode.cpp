//
// Created by hanfuyong on 2022/10/17.
//

#include "StateEstimationNode.h"

namespace Naive_SLAM_ROS{

StateEstimationNode::StateEstimationNode(const std::string& strParamFile):
Node("state_estimation_node"){
    pc_subscriptor_ = this->create_subscription<sensor_msgs::msg::PointCloud>("point_cloud", 100, 
        std::bind(&StateEstimationNode::PointCloudCallback, this, std::placeholders::_1));

    imu_subscriptor_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu0", 100, 
        std::bind(&StateEstimationNode::IMUCallback, this, std::placeholders::_1));
}

void StateEstimationNode::PointCloudCallback(const sensor_msgs::msg::PointCloud::ConstSharedPtr& pc_msg){
    std::vector<long unsigned int> vChainIds;
    // std::vector<cv::Point2f> vPtsUn, vPts, vPtUnOffsets;
    std::vector<cv::Vec3f> vPtsUn;
    std::vector<cv::Vec2f> vPts, vPtUnOffsets;
    for(int i = 0; i < pc_msg->points.size(); i++){
        long unsigned int chainId = pc_msg->channels[0].values[i];
        cv::Vec3f ptUn(pc_msg->points[i].x, pc_msg->points[i].y, pc_msg->points[i].z);
        cv::Vec2f pt(pc_msg->channels[1].values[i], pc_msg->channels[2].values[i]);
        cv::Vec2f ptUnOffset(pc_msg->channels[3].values[i], pc_msg->channels[4].values[i]);
        vChainIds.emplace_back(chainId);
        vPtsUn.emplace_back(ptUn);
        vPts.emplace_back(pt);
        vPtUnOffsets.emplace_back(ptUnOffset);

    }

    Frame frame(pc_msg->header.stamp.nanosec, vChainIds, vPtsUn, vPts, vPtUnOffsets);
    mqFrames.emplace(frame);
}

void StateEstimationNode::IMUCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg){
    double ax = imu_msg->linear_acceleration.x;
    double ay = imu_msg->linear_acceleration.y;
    double az = imu_msg->linear_acceleration.z;
    double wx = imu_msg->angular_velocity.x;
    double wy = imu_msg->angular_velocity.y;
    double wz = imu_msg->angular_velocity.z;

    IMU imu(imu_msg->header.stamp.nanosec, Eigen::Vector3d(ax, ay, az), Eigen::Vector3d(wx, wy, wz));
    
    mqIMUs.emplace(imu);
}

void StateEstimationNode::BindIMUAndImage(){
    while(true){
        if(mqIMUs.empty() || mqFrames.empty()){
            continue;
        }
        if(mqIMUs.back().miTimestamp <= mqFrames.front().miTimestamp){
            // The newest imu in mqIMUs is older than the oldest frame in mqFrames, just wait for more imu
            continue;
        }
        if(mqIMUs.front().miTimestamp <= mqFrames.front().miTimestamp){
            // The oldest imu came after the oldest frame, so the frames that came before imu need to be removed.
            mqFrames.pop();
            continue;
        }

        // Now we get some imus came before the oldest frame and some imus came after the oldest frame
        Frame oneFrame = mqFrames.front();
        mqFrames.pop();
        std::vector<IMU> vIMUs;
        while(mqIMUs.front().miTimestamp < oneFrame.miTimestamp){
            vIMUs.emplace_back(mqIMUs.front());
            mqIMUs.pop();
        }
        vIMUs.emplace_back(mqIMUs.front());
        mqMeasurements.emplace(std::make_pair(oneFrame, vIMUs));
    }
}

void StateEstimationNode::Run(){
    while(!mqMeasurements.empty()){
        std::pair<Frame, std::vector<IMU>> pMeas = mqMeasurements.front();
        mqMeasurements.pop();

    }
}

void StateEstimationNode::ProcessFrame(Frame& frame){
}

} // namespace naive_slam_ros