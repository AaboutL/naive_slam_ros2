#include "FeatureTrackingNode.h"

namespace Naive_SLAM_ROS {

FeatureTrackingNode::FeatureTrackingNode(const std::string& strParamFile) : Node("feature_tracking_node"){
    mpFT = std::make_shared<FeatureTracker>(strParamFile);
    pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud", 1000);
    imshow_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("imshow", 1000);

    im_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/cam0/image_raw", 100, 
        std::bind(&FeatureTrackingNode::FeatureTrackingCallback, this, -1));
}


void FeatureTrackingNode::FeatureTrackingCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const {
    cv_bridge::CvImageConstPtr cvImgPtr = cv_bridge::toCvCopy(img_msg);

}

} // namespace naive_slam_ros