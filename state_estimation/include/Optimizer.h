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
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "Frame.h"
#include "FeatureManager.h"
#include "TypeConverter.h"
#include "GeometryFunc.h"
#include "G2OTypes.h"

#include <rclcpp/rclcpp.hpp>

namespace Naive_SLAM_ROS{

class Optimizer{
public:
    static int PoseOptimize();

    // invert depth optimization, not fully correct
    static int VisualInitBAInvertDepth(Frame& frame1, Frame& frame2, FeatureManager* pFM,
                        std::vector<cv::Vec3f>& vPts3D,
                        std::vector<cv::Vec2f>& vPts2D1, std::vector<cv::Vec2f>& vPts2D2,
                        std::vector<unsigned long>& vChainIds, const cv::Mat& K);

    static int VisualInitBA(Frame& frame1, Frame& frame2, FeatureManager* pFM,
                        std::vector<cv::Vec3f>& vPts3D,
                        std::vector<cv::Vec2f>& vPts2D1, std::vector<cv::Vec2f>& vPts2D2,
                        std::vector<unsigned long>& vChainIds, const cv::Mat& K);
    
    static int VisualBA(std::vector<Frame>& vFrames, FeatureManager* pFM);
};
    
} // namespace Naive_SLAM_ROS


#endif