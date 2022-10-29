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
#include <opencv2/core/eigen.hpp>

#include "TypeConverter.h"

namespace Naive_SLAM_ROS{

class GeometryFunc{
public:
    static Eigen::Vector3d Triangulate(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, 
        const Eigen::Matrix<double, 3, 4>& P1, const Eigen::Matrix<double, 3, 4>& P2);

    static Eigen::Vector2d project(const Eigen::Vector3d& pt3D, const Eigen::Matrix3d& mK);

    static Eigen::Vector2d project2d(const Eigen::Vector3d& v);
    static Eigen::Vector3d unproject2d(const Eigen::Vector2d& v);
    static Eigen::Vector3d invert_depth(const Eigen::Vector3d& v);

    static void SolvePnP(const std::vector<Eigen::Vector3d>& vPts3D, const std::vector<Eigen::Vector2d>& vPts2D,
        const Eigen::Matrix3d& K, Eigen::Matrix3d& R, Eigen::Vector3d& t);
};
    
} // namespace Naive_SLAM_ROS


#endif