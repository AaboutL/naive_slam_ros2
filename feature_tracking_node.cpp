//
// Created by hanfuyong on 2022/10/12.
//

#include <iostream>
#include "FeatureTrackingNode.h"

using namespace Naive_SLAM_ROS

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeatureTrackingNode>());
    rclcpp::shutdown();
    return 0;
}