//
// Created by hanfuyong on 2022/10/22
//

#ifndef NAIVESLAMROS_TYPECONVERTER_H
#define NAIVESLAMROS_TYPECONVERTER_H

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__ // 与上一行一样，但是不能删除
#endif

#include <vector>
#include <eigen3/Eigen/Dense>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <opencv2/opencv.hpp>

namespace Naive_SLAM_ROS{

class TypeConverter{
public:
    static g2o::SE3Quat TtoSE3Quat(const cv::Mat& T){
        Eigen::Matrix3d R;
        R << T.at<float>(0, 0), T.at<float>(0, 1), T.at<float>(0, 2),
             T.at<float>(1, 0), T.at<float>(1, 1), T.at<float>(1, 2),
             T.at<float>(2, 0), T.at<float>(2, 1), T.at<float>(2, 2);
        Eigen::Vector3d t;
        t << T.at<float>(0, 3), T.at<float>(1, 3), T.at<float>(2, 3);

        return g2o::SE3Quat(R, t);
    }
    
    static Eigen::Vector3d VecCVtoEigen(const cv::Mat& vec){
        return Eigen::Vector3d(vec.at<double>(0), vec.at<double>(1), vec.at<double>(2));
    }

    static Eigen::Matrix3d MatCVtoEigen(const cv::Mat& mat){
        Eigen::Matrix3d m;
        m << mat.at<double>(0, 0) , mat.at<double>(0, 1), mat.at<double>(0, 2),
             mat.at<double>(1, 0) , mat.at<double>(1, 1), mat.at<double>(1, 2),
             mat.at<double>(2, 0) , mat.at<double>(2, 1), mat.at<double>(2, 2);
        return m;
    }

    static cv::Mat SE3toT(const g2o::SE3Quat& SE3Quat){
        cv::Mat T = cv::Mat::zeros(4, 4, CV_32F);
        Eigen::Matrix<double,4,4> eigMat = SE3Quat.to_homogeneous_matrix();
        for (int i = 0; i < 4; i++){
            for (int j = 0; j < 4; j++){
                T.at<float>(i, j) = eigMat(i, j);
            }
        }
        return T;
    }

};
    
} // namespace Naive_SLAM_ROS


#endif