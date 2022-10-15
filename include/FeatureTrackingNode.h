//
// Created by hanfuyong on 2022/10/12.
//

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <cv_bridge/cv_bridge.h>

#include "FeatureTracker.h"

using std::placeholders::_1;


namespace Naive_SLAM_ROS {
class FeatureTrackingNode : public rclcpp::Node {
public:
    FeatureTrackingNode(const std::string& strParamFile);

private:
    void FeatureTrackingCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;

private:
    // FeatureTracker* mFT;
    std::shared_ptr<FeatureTracker> mpFT;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr im_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pc_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imshow_publisher_;
};

} // namespace naive_slam_ros