//
// Created by hanfuyong on 2022/10/22
//

#include "Optimizer.h"
#include "utils.h"

namespace Naive_SLAM_ROS{

int Optimizer::VisualInitBAInvertDepth(Frame& frame1, Frame& frame2, FeatureManager* pFM, std::vector<cv::Vec3f>& vPts3D,
                        std::vector<cv::Vec2f>& vPts2D1, std::vector<cv::Vec2f>& vPts2D2,
                        std::vector<unsigned long>& vChainIds, const cv::Mat& K){
    g2o::SparseOptimizer optimizer;
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver =
            std::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr =
            std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver));
    auto *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    optimizer.setAlgorithm(solver);

    float thHuberMono = sqrt(5.991);

    int vertexIdx = 0;
    // add pose vertex
    auto *vSE31 = new g2o::VertexSE3Expmap();
    vSE31->setEstimate(TypeConverter::TtoSE3Quat(frame1.mTcw));
    vSE31->setId(vertexIdx++);
    vSE31->setFixed(true);
    optimizer.addVertex(vSE31);

    auto *vSE32 = new g2o::VertexSE3Expmap();
    vSE32->setEstimate(TypeConverter::TtoSE3Quat(frame2.mTcw));
    vSE32->setId(vertexIdx++);
    optimizer.addVertex(vSE32);

    float sum_diff2 = 0;

    std::vector<std::pair<int, unsigned long>> vVid2Cid;
    std::vector<std::pair<int, int>> vVid2Pid;
    for(int i = 0; i < vPts3D.size(); i++){
        if(vPts3D[i][2] != 0){
            vVid2Cid.emplace_back(std::make_pair(vertexIdx, vChainIds[i]));
            vVid2Pid.emplace_back(std::make_pair(vertexIdx, i));

            auto *vPoint = new g2o::VertexPointXYZ();
            vPoint->setEstimate(GeometryFunc::invert_depth(TypeConverter::VecCVtoEigen(vPts3D[i])));
            vPoint->setId(vertexIdx);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            // g2o::EdgeProjectPSI2UV* e1 = new g2o::EdgeProjectPSI2UV();
            EdgeProjectInvertDepth* e1 = new EdgeProjectInvertDepth();
            e1->resize(3);
            e1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(vertexIdx))); // first is landmark vertex
            e1->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0))); // second is current frame pose vertex
            e1->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0))); // third is reference frame pose vertex
            Eigen::Vector2d obs1;
            obs1 << vPts2D1[i][0], vPts2D1[i][1];
            e1->setMeasurement(obs1);
            e1->information() = Eigen::Matrix2d::Identity();
            g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
            rk1->setDelta(thHuberMono);
            e1->setRobustKernel(rk1);
            e1->setParameterId(0, 0);
            e1->fx = K.at<float>(0, 0);
            e1->fy = K.at<float>(1, 1);
            e1->cx = K.at<float>(0, 2);
            e1->cy = K.at<float>(1, 2);
            optimizer.addEdge(e1);

            EdgeProjectInvertDepth* e2 = new EdgeProjectInvertDepth();
            e2->resize(3);
            e2->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(vertexIdx))); // first is landmark vertex
            e2->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(1))); // second is current frame pose vertex
            e2->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0))); // third is reference frame pose vertex
            Eigen::Vector2d obs2;
            obs2 << vPts2D2[i][0], vPts2D2[i][1];
            e1->setMeasurement(obs2);
            e2->information() = Eigen::Matrix2d::Identity();
            g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
            rk2->setDelta(thHuberMono);
            e2->setRobustKernel(rk2);
            e2->setParameterId(0, 0);
            e2->fx = K.at<float>(0, 0);
            e2->fy = K.at<float>(1, 1);
            e2->cx = K.at<float>(0, 2);
            e2->cy = K.at<float>(1, 2);
            optimizer.addEdge(e2);

            vertexIdx++;

            // calc errors
            Eigen::Vector2d diff = GeometryFunc::project(vSE31->estimate().inverse() *
                            GeometryFunc::invert_depth(vPoint->estimate()), K) - Eigen::Vector2d(vPts2D1[i][0], vPts2D1[i][1]);
            sum_diff2 += diff.dot(diff);

            diff = GeometryFunc::project(vSE32->estimate() *
                            GeometryFunc::invert_depth(vPoint->estimate()), K) - Eigen::Vector2d(vPts2D2[i][0], vPts2D2[i][1]);
            sum_diff2 += diff.dot(diff);
        }
    }
    std::cout << "points square error before optimization*******************************: " << sum_diff2 / vVid2Cid.size() << std::endl;
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(10);

    sum_diff2 = 0;
    for(auto& p: vVid2Pid){
        auto *vPoint = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(p.first));

        Eigen::Vector2d diff = GeometryFunc::project(vSE31->estimate().inverse() *
                        GeometryFunc::invert_depth(vPoint->estimate()), K) - 
                        Eigen::Vector2d(vPts2D1[p.second][0], vPts2D1[p.second][1]);
        sum_diff2 += diff.dot(diff);

        diff = GeometryFunc::project(vSE32->estimate() *
                        GeometryFunc::invert_depth(vPoint->estimate()), K) - 
                        Eigen::Vector2d(vPts2D2[p.second][0], vPts2D2[p.second][1]);
        sum_diff2 += diff.dot(diff);
    }
    std::cout << "points square error after optimization*******************************: " << sum_diff2 / vVid2Cid.size() << std::endl;

    frame1.SetTcw(TypeConverter::SE3toT(vSE31->estimate()));
    frame2.SetTcw(TypeConverter::SE3toT(vSE32->estimate()));

    for(auto& p : vVid2Cid){
        auto vertexId = p.first;
        auto chainId = p.second;
        auto *vPoint = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(vertexId));
        pFM->SetPosition(chainId, TypeConverter::toCvVec(vPoint->estimate()));
    }
}

int Optimizer::VisualInitBA(Frame& frame1, Frame& frame2, FeatureManager* pFM, std::vector<cv::Vec3f>& vPts3D,
                        std::vector<cv::Vec2f>& vPts2D1, std::vector<cv::Vec2f>& vPts2D2,
                        std::vector<unsigned long>& vChainIds, const cv::Mat& K){
    g2o::SparseOptimizer optimizer;
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver =
            std::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr =
            std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver));
    auto *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    optimizer.setAlgorithm(solver);

    float thHuberMono = sqrt(5.991);

    int vertexIdx = 0;
    // add pose vertex
    auto *vSE31 = new g2o::VertexSE3Expmap();
    vSE31->setEstimate(TypeConverter::TtoSE3Quat(frame1.mTcw));
    vSE31->setId(vertexIdx++);
    vSE31->setFixed(true);
    optimizer.addVertex(vSE31);

    auto *vSE32 = new g2o::VertexSE3Expmap();
    vSE32->setEstimate(TypeConverter::TtoSE3Quat(frame2.mTcw));
    vSE32->setId(vertexIdx++);
    optimizer.addVertex(vSE32);

    PrintCvMat("frame1: ", TypeConverter::SE3toT(vSE31->estimate()));
    PrintCvMat("frame2: ", TypeConverter::SE3toT(vSE32->estimate()));
    float sum_diff2 = 0;

    std::vector<std::pair<int, unsigned long>> vVid2Cid;
    std::vector<std::pair<int, int>> vVid2Pid;
    for(int i = 0; i < vPts3D.size(); i++){
        if(vPts3D[i][2] != 0){
            std::cout << vPts2D1[i] << std::endl;
            std::cout << vPts2D2[i] << std::endl;
            vVid2Cid.emplace_back(std::make_pair(vertexIdx, vChainIds[i]));
            vVid2Pid.emplace_back(std::make_pair(vertexIdx, i));

            auto *vPoint = new g2o::VertexPointXYZ();
            vPoint->setEstimate(TypeConverter::VecCVtoEigen(vPts3D[i]));
            vPoint->setId(vertexIdx);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            auto* e1 = new g2o::EdgeSE3ProjectXYZ();
            e1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(vertexIdx))); // first is landmark vertex
            e1->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0))); // second is current frame pose vertex
            Eigen::Vector2d obs1;
            obs1 << vPts2D1[i][0], vPts2D1[i][1];
            e1->setMeasurement(obs1);
            e1->information() = Eigen::Matrix2d::Identity();
            g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
            rk1->setDelta(thHuberMono);
            e1->setRobustKernel(rk1);
            e1->fx = K.at<float>(0, 0);
            e1->fy = K.at<float>(1, 1);
            e1->cx = K.at<float>(0, 2);
            e1->cy = K.at<float>(1, 2);
            optimizer.addEdge(e1);

            auto* e2 = new g2o::EdgeSE3ProjectXYZ();
            e2->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(vertexIdx))); // first is landmark vertex
            e2->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(1))); // second is current frame pose vertex
            Eigen::Vector2d obs2;
            obs2 << vPts2D2[i][0], vPts2D2[i][1];
            e1->setMeasurement(obs2);
            e2->information() = Eigen::Matrix2d::Identity();
            g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
            rk2->setDelta(thHuberMono);
            e2->setRobustKernel(rk2);
            e2->fx = K.at<float>(0, 0);
            e2->fy = K.at<float>(1, 1);
            e2->cx = K.at<float>(0, 2);
            e2->cy = K.at<float>(1, 2);
            optimizer.addEdge(e2);

            vertexIdx++;

            // std::cout << vPts3D[i] << std::endl;
            // calc errors
            // Eigen::Vector2d diff = GeometryFunc::project(vSE31->estimate() * vPoint->estimate(), K) -
            Eigen::Vector2d diff = GeometryFunc::project(vSE31->estimate().map(vPoint->estimate()), K) -
                    Eigen::Vector2d(vPts2D1[i][0], vPts2D1[i][1]);
            sum_diff2 += diff.dot(diff);

            // diff = GeometryFunc::project(vSE32->estimate() * vPoint->estimate(), K) - 
            diff = GeometryFunc::project(vSE32->estimate().map(vPoint->estimate()), K) - 
                    Eigen::Vector2d(vPts2D2[i][0], vPts2D2[i][1]);
            sum_diff2 += diff.dot(diff);
        }
    }
    std::cout << "points square error before optimization*******************************: " << sum_diff2 / vVid2Cid.size() << std::endl;
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(10);

    sum_diff2 = 0;
    for(auto& p: vVid2Pid){
        auto *vPoint = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(p.first));
        std::cout << TypeConverter::toCvVec(vPoint->estimate()) << std::endl;

        // Eigen::Vector2d diff = GeometryFunc::project(vSE31->estimate() * vPoint->estimate(), K) - 
        Eigen::Vector2d diff = GeometryFunc::project(vSE31->estimate().map(vPoint->estimate()), K) - 
                        Eigen::Vector2d(vPts2D1[p.second][0], vPts2D1[p.second][1]);
        sum_diff2 += diff.dot(diff);

        // diff = GeometryFunc::project(vSE32->estimate() * vPoint->estimate(), K) - 
        diff = GeometryFunc::project(vSE32->estimate().map(vPoint->estimate()), K) - 
                        Eigen::Vector2d(vPts2D2[p.second][0], vPts2D2[p.second][1]);
        sum_diff2 += diff.dot(diff);
    }
    std::cout << "points square error after optimization*******************************: " << sum_diff2 / vVid2Cid.size() << std::endl;

    frame1.SetTcw(TypeConverter::SE3toT(vSE31->estimate()));
    frame2.SetTcw(TypeConverter::SE3toT(vSE32->estimate()));

    PrintCvMat("frame1: ", TypeConverter::SE3toT(vSE31->estimate()));
    PrintCvMat("frame2: ", TypeConverter::SE3toT(vSE32->estimate()));

    for(auto& p : vVid2Cid){
        auto vertexId = p.first;
        auto chainId = p.second;
        auto *vPoint = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(vertexId));
        pFM->SetPosition(chainId, TypeConverter::toCvVec(vPoint->estimate()));
    }

}
    
} // namespace Naive_SLAM_ROS
