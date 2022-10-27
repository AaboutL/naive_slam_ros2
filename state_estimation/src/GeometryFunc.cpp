//
// Created by hanfuyong on 2022/10/22
//

#include "GeometryFunc.h"

namespace Naive_SLAM_ROS{

// cv::Mat GeometryFunc::Triangulate(const cv::Vec2f& pt1, const cv::Vec2f& pt2, 
//     const cv::Mat& P1, const cv::Mat& P2){
//     cv::Mat A(4, 4, CV_32F);
//     A.row(0) = pt1[0] * P1.row(2) - P1.row(0);
//     A.row(1) = pt1[1] * P1.row(2) - P1.row(1);
//     A.row(2) = pt2[0] * P2.row(2) - P2.row(0);
//     A.row(3) = pt2[1] * P2.row(2) - P2.row(1);
//     cv::Mat u, w, vt;
//     cv::SVD::compute(A, u, w, vt, cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
//     cv::Mat pt3D = vt.row(3).t();
//     pt3D = pt3D.rowRange(0, 3) / pt3D.at<float>(3);
//     return pt3D;
// }

cv::Vec3f GeometryFunc::Triangulate(const cv::Vec2f& pt1, const cv::Vec2f& pt2, 
    const cv::Mat& P1, const cv::Mat& P2){
    cv::Mat A(4, 4, CV_32F);
    A.row(0) = pt1[0] * P1.row(2) - P1.row(0);
    A.row(1) = pt1[1] * P1.row(2) - P1.row(1);
    A.row(2) = pt2[0] * P2.row(2) - P2.row(0);
    A.row(3) = pt2[1] * P2.row(2) - P2.row(1);
    cv::Mat u, w, vt;
    cv::SVD::compute(A, u, w, vt, cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    cv::Mat tmp = vt.row(3).t();
    return cv::Vec3f(tmp.at<float>(0), tmp.at<float>(1), tmp.at<float>(2)) / tmp.at<float>(3);
}

cv::Vec2f GeometryFunc::project(const cv::Vec3f& pt3D, const cv::Mat& mK){
    return cv::Vec2f(pt3D[0] * mK.at<float>(0, 0) / pt3D[2] + mK.at<float>(0, 2),
                     pt3D[1] * mK.at<float>(1, 1) / pt3D[2] + mK.at<float>(1, 2));
}

// cv::Vec2f GeometryFunc::project(const cv::Mat& pt3D, const cv::Mat& mK){
//     cv::Vec3f tmp(pt3D.at<float>(0), pt3D.at<float>(1), pt3D.at<float>(2));
//     return project(tmp, mK);
// }

Eigen::Vector2d GeometryFunc::project(const Eigen::Vector3d& pt3D, const cv::Mat& mK){
    double u = pt3D(0) * mK.at<float>(0, 0) / pt3D(2) + mK.at<float>(0, 2);
    double v = pt3D(1) * mK.at<float>(1, 1) / pt3D(2) + mK.at<float>(1, 2);
    return Eigen::Vector2d(u, v);
}

Eigen::Vector2d GeometryFunc::project2d(const Eigen::Vector3d& v){
    Eigen::Vector2d res;
    res(0) = v(0) / v(2);
    res(1) = v(1) / v(2);
    return res;
}

Eigen::Vector3d GeometryFunc::unproject2d(const Eigen::Vector2d& v){
    Eigen::Vector3d res;
    res(0) = v(0);
    res(1) = v(1);
    res(2) = 1;
    return res;
}

Eigen::Vector3d GeometryFunc::invert_depth(const Eigen::Vector3d& x){
    return unproject2d(x.head<2>()) / x[2];
}
    
} // namespace Naive_SLAM_ROS
