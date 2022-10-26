#include "FeatureTrackingNode.h"

namespace Naive_SLAM_ROS {

FeatureTrackingNode::FeatureTrackingNode(const std::string& strParamFile) : 
    mnPubNum(0), Node("feature_tracking_node"){
    mpFT = std::make_shared<FeatureTracker>(strParamFile);
    pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud", 1000);
    visTrack_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("vis_track", 1000);

    im_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/cam0/image_raw", 100, 
        std::bind(&FeatureTrackingNode::FeatureTrackingCallback, this, std::placeholders::_1));
}


void FeatureTrackingNode::FeatureTrackingCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) {
    cv_bridge::CvImageConstPtr cvImgPtr = cv_bridge::toCvCopy(img_msg);

    std::vector<unsigned long> vChainIds;
    std::vector<cv::Point2f> vPtsUn;
    std::vector<cv::Point2f> vPts;
    std::vector<cv::Point2f> vPtUnOffsets;
    std::vector<int> vChainLens;

    mpFT->Track(cvImgPtr->image, vChainIds, vPtsUn, vPts, vPtUnOffsets, vChainLens);

    sensor_msgs::msg::PointCloud pcMsg = sensor_msgs::msg::PointCloud();
    pcMsg.header = img_msg->header;
    pcMsg.header.frame_id = "world";

    sensor_msgs::msg::ChannelFloat32 chainIds;
    sensor_msgs::msg::ChannelFloat32 pts_u;
    sensor_msgs::msg::ChannelFloat32 pts_v;
    sensor_msgs::msg::ChannelFloat32 pts_u_offset;
    sensor_msgs::msg::ChannelFloat32 pts_v_offset;

    for(size_t i = 0; i < vChainIds.size(); i++){
        if(vChainLens[i] > 1){
            geometry_msgs::msg::Point32 ptUn;
            ptUn.x = vPtsUn[i].x;
            ptUn.y = vPtsUn[i].y;
            ptUn.z = 1;

            pcMsg.points.emplace_back(ptUn);
            chainIds.values.emplace_back(vChainIds[i]);
            pts_u.values.emplace_back(vPts[i].x);
            pts_v.values.emplace_back(vPts[i].y);
            pts_u_offset.values.emplace_back(vPtUnOffsets[i].x);
            pts_v_offset.values.emplace_back(vPtUnOffsets[i].y);
        }
    }
    pcMsg.channels.emplace_back(chainIds);
    pcMsg.channels.emplace_back(pts_u);
    pcMsg.channels.emplace_back(pts_v);
    pcMsg.channels.emplace_back(pts_u_offset);
    pcMsg.channels.emplace_back(pts_v_offset);

    pc_publisher_->publish(pcMsg);

    if(mnPubNum != 0){
        cv::Mat visTrack = mpFT->DrawMatches();
        sensor_msgs::msg::Image::SharedPtr visTrackMsgPtr = 
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", visTrack).toImageMsg();
        visTrack_publisher_->publish(*visTrackMsgPtr.get());
    }
    mnPubNum++;
}

} // namespace naive_slam_ros