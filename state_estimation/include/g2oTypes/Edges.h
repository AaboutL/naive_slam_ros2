//
// Created by hanfuyong on 2022/11/09
//

#ifndef NAIVESLAMROS_EDGES_H
#define NAIVESLAMROS_EDGES_H

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__ // same but can not remove
#endif

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <sophus/se3.hpp>

#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>

#include "g2oTypes/Vertices.h"
#include "IMU.h"

namespace Naive_SLAM_ROS{

typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 15, 15> Matrix15d;

class EdgeInertialGS: public g2o::BaseMultiEdge<9, Vector9d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeInertialGS(Preintegrator* pPreint);
    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const {return false;}

    void computeError();
    virtual void linearizeOplus();

public:
    Eigen::Matrix3d mJRbg;
    Eigen::Matrix3d mJVba;
    Eigen::Matrix3d mJVbg;
    Eigen::Matrix3d mJPba;
    Eigen::Matrix3d mJPbg;

    Preintegrator* mpPreint;
    Eigen::Vector3d mGI, mG;
    double mdT;
};
    
////////////////////////////////////////////////////////////////////////////////////////////////////////

class EdgePriorAccBias : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexAccBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePriorAccBias(const Eigen::Vector3d &bprior_) : bprior(bprior_) {}

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    void computeError()
    {
        const VertexAccBias *VAB = static_cast<const VertexAccBias *>(_vertices[0]);
        _error = bprior - VAB->estimate();
    }

    virtual void linearizeOplus(){
        _jacobianOplusXi.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix<double, 3, 3> GetHessian()
    {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }

    const Eigen::Vector3d bprior;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////

class EdgePriorGyrBias : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexGyrBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePriorGyrBias(const Eigen::Vector3d &bprior_) : bprior(bprior_) {}

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    void computeError()
    {
        const VertexGyrBias *VGB = static_cast<const VertexGyrBias *>(_vertices[0]);
        _error = bprior - VGB->estimate();
    }

    virtual void linearizeOplus(){
        _jacobianOplusXi.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix<double, 3, 3> GetHessian()
    {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }

    const Eigen::Vector3d bprior;
};

} // namespace Naive_SLAM_ROS


#endif