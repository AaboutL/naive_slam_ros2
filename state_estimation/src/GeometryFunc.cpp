//
// Created by hanfuyong on 2022/10/22
//

#include "GeometryFunc.h"

namespace Naive_SLAM_ROS{

Eigen::Vector3d GeometryFunc::Triangulate(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, 
    const Eigen::Matrix<double, 3, 4>& P1, const Eigen::Matrix<double, 3, 4>& P2){
    Eigen::Matrix4d A;
    A.row(0) = pt1[0] * P1.row(2) - P1.row(0);
    A.row(1) = pt1[1] * P1.row(2) - P1.row(1);
    A.row(2) = pt2[0] * P2.row(2) - P2.row(0);
    A.row(3) = pt2[1] * P2.row(2) - P2.row(1);

    Eigen::Vector3d pt3D;
    Eigen::Vector4d triangulated_point;
	triangulated_point = A.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
	pt3D(0) = triangulated_point(0) / triangulated_point(3);
	pt3D(1) = triangulated_point(1) / triangulated_point(3);
	pt3D(2) = triangulated_point(2) / triangulated_point(3);
    return pt3D;
}

Eigen::Vector2d GeometryFunc::project(const Eigen::Vector3d& pt3D, const Eigen::Matrix3d& mK){
    return Eigen::Vector2d(pt3D[0] * mK(0, 0) / pt3D[2] + mK(0, 2),
                           pt3D[1] * mK(1, 1) / pt3D[2] + mK(1, 2));
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

void GeometryFunc::SolvePnP(const std::vector<Eigen::Vector3d>& vPts3D, const std::vector<Eigen::Vector2d>& vPts2D,
        const Eigen::Matrix3d& K, Eigen::Matrix3d& R21, Eigen::Vector3d& t21){

    std::vector<cv::Vec3f> cvPts3D(vPts3D.size()); 
    std::vector<cv::Vec2f> cvPts2D(vPts2D.size());
    for(int i = 0; i < vPts2D.size(); i++){
        cvPts3D[i] = cv::Vec3f(vPts3D[i][0], vPts3D[i][1], vPts3D[i][2]);
        cvPts2D[i] = cv::Vec2f(vPts2D[i][0], vPts2D[i][1]);
    }
    cv::Mat cvr21, cvt21, cvR21, inliers, cvK;
    cv::eigen2cv(K, cvK);
    cv::solvePnPRansac(cvPts3D, cvPts2D, cvK, cv::Mat::zeros(4, 1, CV_32F),
                        cvr21, cvt21, false, 100, 2, 0.99, inliers, cv::SOLVEPNP_EPNP);
    
    cv::Rodrigues(cvr21, cvR21);
    R21 = TypeConverter::MatCVtoEigen(cvR21);
    t21 = TypeConverter::VecCVtoEigen(cvt21);
}
    
} // namespace Naive_SLAM_ROS
