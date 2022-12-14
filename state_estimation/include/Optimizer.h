//
// Created by hanfuyong on 2022/10/22
//

#ifndef NAIVESLAMROS_OPTIMIZER_H
#define NAIVESLAMROS_OPTIMIZER_H

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__ // 与上一行一样，但是不能删除
#endif

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "Frame.h"
#include "FeatureManager.h"
#include "TypeConverter.h"
#include "GeometryFunc.h"
#include "G2OTypes.h"

#include "Vertices.h"
#include "Edges.h"

#include <rclcpp/rclcpp.hpp>

#define EPSILON 0.000000000001

namespace Naive_SLAM_ROS{

class Optimizer{
public:
    static int PoseOptimize();

    // invert depth optimization, not fully correct
    static int VisualOnlyInitBAInvertDepth(Frame& frame1, Frame& frame2, 
                        std::shared_ptr<FeatureManager> pFM,
                        std::vector<Eigen::Vector3d>& vPts3D,
                        std::vector<Eigen::Vector2d>& vPts2D1, std::vector<Eigen::Vector2d>& vPts2D2,
                        std::vector<unsigned long>& vChainIds, const Eigen::Matrix3d& K);

    static int VisualOnlyInitBA(Frame* frame1, Frame* frame2, 
                        // std::shared_ptr<FeatureManager>& pFM,
                        FeatureManager* pFM,
                        std::vector<Eigen::Vector3d>& vPts3D,
                        std::vector<Eigen::Vector2d>& vPts2D1, std::vector<Eigen::Vector2d>& vPts2D2,
                        std::vector<unsigned long>& vChainIds, const Eigen::Matrix3d& K);
    
    static int VisualOnlyBA(std::vector<Frame*>& vpFrames, 
                        // std::shared_ptr<FeatureManager>& pFM, 
                        FeatureManager* pFM, 
                        const Eigen::Matrix3d& K);
    
    static int InertialOnlyBA(std::vector<Frame*>& vpFrames, Eigen::Matrix3d& Rwg, Eigen::Vector3d& gyrBias,
                              Eigen::Vector3d& accBias, double& scale, double priorAcc, double priorGyr);

    static int VisualInertialInitBA(std::vector<Frame*>& vpFrames, FeatureManager* pFM, const Eigen::Matrix3d& K, Eigen::Vector3d& gyrBias,
                                    Eigen::Vector3d& accBias, double priorAcc, double priorGyr);

    // optType: 1 for frame; 2 for KeyFrame
    static int VisualInertialOptimize(std::vector<Frame*>& vpFrames, FeatureManager* pFM, const Eigen::Matrix3d& K, bool bNotKeyFrame);
};
    
} // namespace Naive_SLAM_ROS


#endif