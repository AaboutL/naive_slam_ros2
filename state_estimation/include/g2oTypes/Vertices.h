//
// Created by hanfuyong on 2022/11/08
//

#ifndef NAIVESLAMROS_VERTICES_H
#define NAIVESLAMROS_VERTICES_H

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__ // same but can not remove
#endif

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <sophus/se3.hpp>
#include <g2o/core/base_vertex.h>

namespace Naive_SLAM_ROS
{

class ImuCamPose{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuCamPose(){}
    ImuCamPose(const Sophus::SE3d& Tcb, const Sophus::SE3d& Tcw);
    void Update(const double* pUpdate);

public:
    Sophus::SE3d mTcb;
    Sophus::SE3d mTbc;
    Sophus::SE3d mTcw;
    Sophus::SE3d mTwb;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////

class VertexPose : public g2o::BaseVertex<6, ImuCamPose>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose(){}
    VertexPose(const Sophus::SE3d& Tcb, const Sophus::SE3d& Tcw){
        setEstimate(ImuCamPose(Tcb, Tcw));
    }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    virtual void setToOriginImpl(){}
    virtual void oplusImpl(const double* delta){
        _estimate.Update(delta);
        updateCache();
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////

class VertexVelocity : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexVelocity() {}
    VertexVelocity(const Eigen::Vector3d& vel){
        setEstimate(vel);
    }

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    virtual void setToOriginImpl(){}

    virtual void oplusImpl(const double *delta){
        Eigen::Vector3d dv;
        dv << delta[0], delta[1], delta[2];
        setEstimate(estimate() + dv);
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////

class VertexGyrBias : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGyrBias() {}
    VertexGyrBias(const Eigen::Vector3d& bias){
        setEstimate(bias);
    }

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    virtual void setToOriginImpl()
    {
    }

    virtual void oplusImpl(const double *delta)
    {
        Eigen::Vector3d dbg;
        dbg << delta[0], delta[1], delta[2];
        setEstimate(estimate() + dbg);
    }
};
    
////////////////////////////////////////////////////////////////////////////////////////////////////////

class VertexAccBias : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexAccBias() {}
    VertexAccBias(const Eigen::Vector3d& bias){
        setEstimate(bias);
    }

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    virtual void setToOriginImpl(){}

    virtual void oplusImpl(const double *delta)
    {
        Eigen::Vector3d dba;
        dba << delta[0], delta[1], delta[2];
        setEstimate(estimate() + dba);
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////

class GDirection
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GDirection() {}
    GDirection(const Eigen::Matrix3d& Rwg) : mRwg(Rwg) {}

    void Update(const double *pu)
    {
        mRwg = mRwg * Sophus::SO3d::exp(Eigen::Vector3d(pu[0], pu[1], 0.0)).matrix();
    }

    Eigen::Matrix3d mRwg, mRgw;

    int its;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////

class VertexGDir : public g2o::BaseVertex<2, GDirection>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGDir() {}
    VertexGDir(const Eigen::Matrix3d& Rwg)
    {
        setEstimate(GDirection(Rwg));
    }

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    virtual void setToOriginImpl()
    {
    }

    virtual void oplusImpl(const double *delta)
    {
        _estimate.Update(delta);
        updateCache();
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////

class VertexScale : public g2o::BaseVertex<1, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexScale()
    {
        setEstimate(1.0);
    }
    VertexScale(double s)
    {
        setEstimate(s);
    }

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    virtual void setToOriginImpl()
    {
        setEstimate(1.0);
    }

    virtual void oplusImpl(const double *delta)
    {
        setEstimate(estimate() * exp(*delta));
    }
};

} // namespace Naive_SLAM_ROS


#endif