//
// Created by hanfuyong on 2022/10/12.
//

#include <iostream>
#include "FeatureTrackingNode.h"

using namespace Naive_SLAM_ROS;

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    std::string sParamPath = "/home/parallels/workspace/naive_slam_ros2_ws/src/config/config.yaml";
    rclcpp::spin(std::make_shared<FeatureTrackingNode>(sParamPath));
    rclcpp::shutdown();
    return 0;
}