//
// Created by hanfuyong on 2022/10/22
//

#ifndef NAIVESLAMROS_GEOMETRYFUNC_H
#define NAIVESLAMROS_GEOMETRYFUNC_H

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__ // 与上一行一样，但是不能删除
#endif

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

namespace Naive_SLAM_ROS{

class GeometryFunc{
public:
    static cv::Vec3f Triangulate(const cv::Vec2f& pt1, const cv::Vec2f& pt2, 
        const cv::Mat& P1, const cv::Mat& P2);

    static cv::Vec2f project(const cv::Vec3f& pt3D, const cv::Mat& mK);
    // static cv::Vec2f project(const cv::Mat& pt3D, const cv::Mat& mK);
    static Eigen::Vector2d project(const Eigen::Vector3d& pt3D, const cv::Mat& mK);

    static Eigen::Vector2d project2d(const Eigen::Vector3d& v);
    static Eigen::Vector3d unproject2d(const Eigen::Vector2d& v);
    static Eigen::Vector3d invert_depth(const Eigen::Vector3d& v);
};
    
} // namespace Naive_SLAM_ROS


#endif