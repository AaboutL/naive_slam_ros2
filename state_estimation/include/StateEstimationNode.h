//
// Created by hanfuyong on 2022/10/17.
//

#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>

#include "Frame.h"
#include "IMU.h"
#include "StateEstimator.h"

namespace Naive_SLAM_ROS {

class StateEstimationNode : public rclcpp::Node {
public:
    StateEstimationNode(const std::string& strParamFile);

private:
    void PointCloudCallback(const sensor_msgs::msg::PointCloud::ConstSharedPtr& pc_msg);
    void IMUCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg);
    std::vector<std::pair<Frame, std::vector<IMU>>> BindIMUAndImage();
    void Run();
    void ProcessFrame(Frame& frame);

private:

    std::queue<Frame> mqFrames;
    std::queue<IMU> mqIMUs;

    StateEstimator* mpEstimator;

    std::mutex mMutexBuffer;
    std::condition_variable mConditionVar;
    std::thread mtStateEstimation;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr pc_subscriptor_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriptor_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr kf_pose_publisher_;
};


} // naive_slam_ros

#endif