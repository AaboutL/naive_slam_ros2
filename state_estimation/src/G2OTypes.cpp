//
// Created by hanfuyong on 2022/10/24
//

#include "G2OTypes.h"

namespace Naive_SLAM_ROS{

EdgeProjectInvertDepth::EdgeProjectInvertDepth(){}

bool EdgeProjectInvertDepth::write(std::ostream &os) const {
  writeParamIds(os);
  g2o::internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

bool EdgeProjectInvertDepth::read(std::istream &is) {
  readParamIds(is);
  g2o::internal::readVector(is, _measurement);
  return readInformationMatrix(is);
}

void EdgeProjectInvertDepth::computeError() {
    const g2o::VertexPointXYZ *psi = vertexXn<0>();
    const g2o::VertexSE3Expmap *T_p_from_world = vertexXn<1>();
    const g2o::VertexSE3Expmap *T_anchor_from_world = vertexXn<2>();

    g2o::Vector2 obs(_measurement);
    _error = obs - cam_project(T_p_from_world->estimate() *
                                T_anchor_from_world->estimate().inverse() *
                                g2o::internal::invert_depth(psi->estimate()));
}

void EdgeProjectInvertDepth::linearizeOplus() {
  g2o::VertexPointXYZ *vpoint = vertexXn<0>();
  g2o::Vector3 psi_a = vpoint->estimate();
  g2o::VertexSE3Expmap *vpose = vertexXn<1>();
  g2o::SE3Quat T_cw = vpose->estimate();
  g2o::VertexSE3Expmap *vanchor = vertexXn<2>();

  g2o::SE3Quat A_aw = vanchor->estimate();
  g2o::SE3Quat T_ca = T_cw * A_aw.inverse();
  g2o::Vector3 x_a = g2o::internal::invert_depth(psi_a);
  g2o::Vector3 y = T_ca * x_a;
  Eigen::Matrix<number_t, 2, 3, Eigen::ColMajor> Jcam = d_proj_d_y(fx, fy, y);

  auto &jacobianOplus0 = std::get<0>(this->_jacobianOplus);
  auto &jacobianOplus1 = std::get<1>(this->_jacobianOplus);
  auto &jacobianOplus2 = std::get<2>(this->_jacobianOplus);
  jacobianOplus0 = -Jcam * g2o::internal::d_Tinvpsi_d_psi(T_ca, psi_a);
  jacobianOplus1 = -Jcam * g2o::internal::d_expy_d_y(y);
  jacobianOplus2 =
      Jcam * T_ca.rotation().toRotationMatrix() * g2o::internal::d_expy_d_y(x_a);
}

g2o::Vector2 EdgeProjectInvertDepth::cam_project(const g2o::Vector3 &trans_xyz) const {
    g2o::Vector2 proj = g2o::project(trans_xyz);
    g2o::Vector2 res;
    res[0] = proj[0] * fx + cx;
    res[1] = proj[1] * fy + cy;
    return res;
}


} // namespace Naive_SLAM_ROS
