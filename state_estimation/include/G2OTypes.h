//
// Created by hanfuyong on 2022/10/24
//

#ifndef NAIVESLAMROS_G2OTYPES_H
#define NAIVESLAMROS_G2OTYPES_H

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__ // 与上一行一样，但是不能删除
#endif

#include <g2o/core/base_fixed_sized_edge.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/se3_ops.h>
#include <g2o/types/sba/sba_utils.h>

namespace Naive_SLAM_ROS{

inline Eigen::Matrix<number_t, 2, 3, Eigen::ColMajor> d_proj_d_y(
    const number_t& fx, const number_t& fy, const g2o::Vector3& xyz) {
  number_t z_sq = xyz[2] * xyz[2];
  Eigen::Matrix<number_t, 2, 3, Eigen::ColMajor> J;
  J << fx / xyz[2], 0, -(fx * xyz[0]) / z_sq, 0, fy / xyz[2], -(fy * xyz[1]) / z_sq;
  return J;
}

class EdgeProjectInvertDepth : public g2o::BaseFixedSizedEdge<2, 
    g2o::Vector2, g2o::VertexPointXYZ, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeProjectInvertDepth();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void computeError();
    virtual void linearizeOplus();

    g2o::Vector2 cam_project(const g2o::Vector3 &trans_xyz) const;

    double fx, fy, cx, cy;
};
    
} // namespace Naive_SLAM_ROS


#endif