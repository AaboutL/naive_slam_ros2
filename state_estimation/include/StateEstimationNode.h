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

#include <queue>

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
    void BindIMUAndImage();
    void Run();
    void ProcessFrame(Frame& frame);

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr pc_subscriptor_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriptor_;

    std::queue<Frame> mqFrames;
    std::queue<IMU> mqIMUs;
    std::queue<std::pair<Frame, std::vector<IMU>>> mqMeasurements;

};


} // naive_slam_ros

#endif