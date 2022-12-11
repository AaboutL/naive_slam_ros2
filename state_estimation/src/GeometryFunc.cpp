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

    std::vector<cv::Vec3d> cvPts3D(vPts3D.size()); 
    std::vector<cv::Vec2d> cvPts2D(vPts2D.size());
    for(int i = 0; i < vPts2D.size(); i++){
        cvPts3D[i] = cv::Vec3d(vPts3D[i][0], vPts3D[i][1], vPts3D[i][2]);
        cvPts2D[i] = cv::Vec2d(vPts2D[i][0], vPts2D[i][1]);
    }
    cv::Mat cvr21, cvt21, cvR21, inliers, cvK;
    cv::eigen2cv(K, cvK);
    cv::eigen2cv(t21, cvt21);
    cv::eigen2cv(R21, cvR21);
    cv::Rodrigues(cvR21, cvr21);
    // cv::solvePnPRansac(cvPts3D, cvPts2D, cvK, cv::Mat::zeros(4, 1, CV_32F),
    //                     cvr21, cvt21, false, 100, 2, 0.99, inliers, cv::SOLVEPNP_EPNP);
    cv::solvePnPRansac(cvPts3D, cvPts2D, cvK, cv::Mat::zeros(4, 1, CV_32F),
                        cvr21, cvt21, true, 100, 2, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);
    
    cv::Rodrigues(cvr21, cvR21);
    cv::cv2eigen(cvR21, R21);
    cv::cv2eigen(cvt21, t21);
}
    
std::vector<Eigen::Vector3d> GeometryFunc::TriangulateTwoFrame(const Eigen::Matrix3d& Rcw1, const Eigen::Vector3d& tcw1,
        const Eigen::Matrix3d& Rcw2, const Eigen::Vector3d& tcw2, const Eigen::Matrix3d& K,
        const std::vector<Eigen::Vector2d>& vPts2D1, const std::vector<Eigen::Vector2d>& vPts2D2,
        const std::vector<unsigned long>& vChainIds){

    std::cout << "[GeometryFunc::TriangulateTwoFrame] Start" << std::endl;
    std::vector<Eigen::Vector3d> vPts3D(vPts2D1.size(), Eigen::Vector3d(0, 0, 0));
    Eigen::Matrix<double, 3, 4> P1, P2;
    P1.block<3, 3>(0, 0) = Rcw1;
    P1.block<3, 1>(0, 3) = tcw1;
    P1 = K * P1;
    P2.block<3, 3>(0, 0) = Rcw2;
    P2.block<3, 1>(0, 3) = tcw2;
    P2 = K * P2;

    int rej_z1 = 0;
    int rej_z2 = 0;
    int rej_err1 = 0;
    int rej_err2 = 0;
    for(int j = 0; j < vPts2D1.size(); j++){
        Eigen::Vector2d pt1 = vPts2D1[j], pt2 = vPts2D2[j];
        Eigen::Vector3d pt3DW = GeometryFunc::Triangulate(pt1, pt2, P1, P2);

        auto pt3DC1 = Rcw1 * pt3DW + tcw1;
        if(!finite(pt3DC1[0]) || !finite(pt3DC1[1]) || !finite(pt3DC1[2]) || pt3DC1[2] <= 0){
            rej_z1++;
            continue;
        }

        auto pt3DC2 = Rcw2 * pt3DW + tcw2;
        if(!finite(pt3DC2[0]) || !finite(pt3DC2[1]) || !finite(pt3DC2[2]) || pt3DC2[2] <= 0){
            rej_z2++;
            continue;
        }

        Eigen::Vector2d uv1 = GeometryFunc::project(pt3DC1, K);
        auto err1 = uv1 - pt1;
        auto err12 = err1.dot(err1);
        if(err12 > 5.991){
            rej_err1++;
            continue;
        }

        Eigen::Vector2d uv2 = GeometryFunc::project(pt3DC2, K);
        auto err2 = uv2 - pt2;
        auto err22 = err2.dot(err2);
        if(err22 > 5.991){
            rej_err2++;
            continue;
        }

        vPts3D[j] = pt3DW;
        // mpFM->SetWorldPos(vChainIds[j], pt3DW);
    }

    std::cout << "[GeometryFunc::TriangulateTwoFrame] Done" << std::endl;
    return vPts3D;
}


Eigen::Vector3d GeometryFunc::R2ypr(const Eigen::Matrix3d& R){
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
}

} // namespace Naive_SLAM_ROS
