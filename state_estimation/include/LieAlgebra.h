//
// Created by hanfuyong on 2022/11/03
//

#ifndef NAIVESLAMROS_LIEALGEBRA_H
#define NAIVESLAMROS_LIEALGEBRA_H

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__ // same but can not remove
#endif

#include <eigen3/Eigen/Dense>

namespace Naive_SLAM_ROS{

namespace LieAlg{

Eigen::Matrix3d hat(const Eigen::Vector3d& v){
    Eigen::Matrix3d v_hat;
    v_hat << 0, -v[2], v[1],
             v[2], 0, -v[0],
             -v[1], v[0], 0;
    return v_hat;
}

Eigen::Matrix3d Exp(const Eigen::Vector3d& v){
    Eigen::Matrix3d R;
    double n2 = v.dot(v);
    double n = sqrt(n2);
    Eigen::Matrix3d v_hat = hat(v);
    if(n < 1e-4){
        R = Eigen::Matrix3d::Identity() + v_hat;
    }
    else{
        R = Eigen::Matrix3d::Identity() + std::sin(n) * v_hat / n + (1 - cos(n)) * v_hat * v_hat / n2;
    }
    return R;
}


} // namespace LieAlg{

} // namespace Naive_SLAM_ROS


#endif