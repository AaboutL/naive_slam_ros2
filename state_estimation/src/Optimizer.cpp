//
// Created by hanfuyong on 2022/10/22
//

#include "Optimizer.h"
#include "utils.h"

namespace Naive_SLAM_ROS{

int Optimizer::VisualInitBAInvertDepth(Frame& frame1, Frame& frame2, std::shared_ptr<FeatureManager> pFM,
                        std::vector<Eigen::Vector3d>& vPts3D,
                        std::vector<Eigen::Vector2d>& vPts2D1, std::vector<Eigen::Vector2d>& vPts2D2,
                        std::vector<unsigned long>& vChainIds, const Eigen::Matrix3d& K){
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
    // vSE31->setEstimate(TypeConverter::TtoSE3Quat(frame1.mTcw));
    vSE31->setEstimate(g2o::SE3Quat(frame1.mRcw, frame1.mtcw));
    vSE31->setId(vertexIdx++);
    vSE31->setFixed(true);
    optimizer.addVertex(vSE31);

    auto *vSE32 = new g2o::VertexSE3Expmap();
    // vSE32->setEstimate(TypeConverter::TtoSE3Quat(frame2.mTcw));
    vSE32->setEstimate(g2o::SE3Quat(frame2.mRcw, frame2.mtcw));
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
            vPoint->setEstimate(GeometryFunc::invert_depth(vPts3D[i]));
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
            e1->fx = K(0, 0);
            e1->fy = K(1, 1);
            e1->cx = K(0, 2);
            e1->cy = K(1, 2);
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
            e2->fx = K(0, 0);
            e2->fy = K(1, 1);
            e2->cx = K(0, 2);
            e2->cy = K(1, 2);
            optimizer.addEdge(e2);

            vertexIdx++;
        }
    }
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(10);


    frame1.SetTcw(vSE31->estimate().to_homogeneous_matrix());
    frame2.SetTcw(vSE32->estimate().to_homogeneous_matrix());

    for(auto& p : vVid2Cid){
        auto vertexId = p.first;
        auto chainId = p.second;
        auto *vPoint = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(vertexId));
        pFM->SetChainPosition(chainId, vPoint->estimate());
        pFM->SetChainGood(chainId, true);
    }
}

int Optimizer::VisualInitBA(Frame& frame1, Frame& frame2, 
                        std::shared_ptr<FeatureManager> pFM, std::vector<Eigen::Vector3d>& vPts3D,
                        std::vector<Eigen::Vector2d>& vPts2D1, std::vector<Eigen::Vector2d>& vPts2D2,
                        std::vector<unsigned long>& vChainIds, const Eigen::Matrix3d& K){
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
    vSE31->setEstimate(g2o::SE3Quat(frame1.mRcw, frame1.mtcw));
    vSE31->setId(vertexIdx++);
    vSE31->setFixed(true);
    optimizer.addVertex(vSE31);

    auto *vSE32 = new g2o::VertexSE3Expmap();
    vSE32->setEstimate(g2o::SE3Quat(frame2.mRcw, frame2.mtcw));
    vSE32->setId(vertexIdx++);
    optimizer.addVertex(vSE32);

    float sum_diff2 = 0;
    std::vector<std::pair<int, unsigned long>> vVid2Cid;
    std::vector<std::pair<int, int>> vVid2Pid;
    for(int i = 0; i < vPts3D.size(); i++){
        if(abs(vPts3D[i][2]) > EPSILON && pFM->IsChainPosSet(vChainIds[i])){
            vVid2Cid.emplace_back(std::make_pair(vertexIdx, vChainIds[i]));
            vVid2Pid.emplace_back(std::make_pair(vertexIdx, i));

            auto *vPoint = new g2o::VertexPointXYZ();
            vPoint->setEstimate(vPts3D[i]);
            vPoint->setId(vertexIdx);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            auto* e1 = new g2o::EdgeSE3ProjectXYZ();
            e1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(vertexIdx))); // first is landmark vertex
            e1->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0))); // second is current frame pose vertex
            e1->setMeasurement(vPts2D1[i]);
            e1->information() = Eigen::Matrix2d::Identity();
            g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
            rk1->setDelta(thHuberMono);
            e1->setRobustKernel(rk1);
            e1->fx = K(0, 0);
            e1->fy = K(1, 1);
            e1->cx = K(0, 2);
            e1->cy = K(1, 2);
            optimizer.addEdge(e1);

            auto* e2 = new g2o::EdgeSE3ProjectXYZ();
            e2->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(vertexIdx))); // first is landmark vertex
            e2->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(1))); // second is current frame pose vertex
            e2->setMeasurement(vPts2D2[i]);
            e2->information() = Eigen::Matrix2d::Identity();
            g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
            rk2->setDelta(thHuberMono);
            e2->setRobustKernel(rk2);
            e2->fx = K(0, 0);
            e2->fy = K(1, 1);
            e2->cx = K(0, 2);
            e2->cy = K(1, 2);
            optimizer.addEdge(e2);

            vertexIdx++;

            // calc errors
            Eigen::Vector2d diff = GeometryFunc::project(vSE31->estimate().map(vPoint->estimate()), K) - vPts2D1[i];
            sum_diff2 += diff.dot(diff);

            diff = GeometryFunc::project(vSE32->estimate().map(vPoint->estimate()), K) - vPts2D2[i];
            sum_diff2 += diff.dot(diff);
        }
    }
    std::cout << "points square error before optimization: " << sum_diff2 / vVid2Cid.size() << std::endl;
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(10);

    sum_diff2 = 0;
    for(auto& p: vVid2Pid){
        auto *vPoint = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(p.first));

        Eigen::Vector2d diff = GeometryFunc::project(vSE31->estimate().map(vPoint->estimate()), K) - vPts2D1[p.second];
        sum_diff2 += diff.dot(diff);

        diff = GeometryFunc::project(vSE32->estimate().map(vPoint->estimate()), K) - vPts2D2[p.second];
        sum_diff2 += diff.dot(diff);
    }
    std::cout << "points square error after optimization: " << sum_diff2 / vVid2Cid.size() << std::endl;

    frame1.SetTcw(vSE31->estimate().to_homogeneous_matrix());
    frame2.SetTcw(vSE32->estimate().to_homogeneous_matrix());

    for(auto& p : vVid2Cid){
        auto vertexId = p.first;
        auto chainId = p.second;
        auto *vPoint = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(vertexId));
        pFM->SetChainPosition(chainId, vPoint->estimate());
        pFM->SetChainGood(chainId, true);
    }
    std::cout << "[Optimizer::VisualInitBA] finished" << std::endl;
    return 1;
}

int Optimizer::VisualBA(std::vector<Frame>& vFrames, std::shared_ptr<FeatureManager> pFM, const Eigen::Matrix3d& K){
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
    std::vector<g2o::VertexSE3Expmap*> vPoses;
    for(int i = 0; i < vFrames.size(); i++){
        auto *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(g2o::SE3Quat(vFrames[i].mRcw, vFrames[i].mtcw));
        vSE3->setFixed(vertexIdx == 0);
        vSE3->setId(vertexIdx++);
        optimizer.addVertex(vSE3);
        vPoses.emplace_back(vSE3);
    }

    // add 3D point vertex
    double sum_diff2 = 0;
    double sum_chi2 = 0;
    std::vector<std::pair<g2o::EdgeSE3ProjectXYZ*, std::vector<int>>> vEdges;
    std::vector<g2o::VertexPointXYZ*> vPoints;
    std::vector<Eigen::Vector2d> vObs;
    for(auto& [id, chain] : pFM->GetChains()){
        if(!chain.mbPosSet)
            continue;
        auto *vPoint = new g2o::VertexPointXYZ();
        vPoint->setEstimate(chain.mWorldPos);
        vPoint->setId(vertexIdx);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        vPoints.emplace_back(vPoint);

        for(int i = 0; i < chain.mvFeatures.size(); i++){
            auto* e = new g2o::EdgeSE3ProjectXYZ();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(vertexIdx))); 
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                optimizer.vertex(chain.mStartIdx + i))); 
            Eigen::Vector2d obs(chain.mvFeatures[i].mPtUn[0], chain.mvFeatures[i].mPtUn[1]);
            vObs.emplace_back(obs);
            e->setMeasurement(obs);
            e->information() = Eigen::Matrix2d::Identity();
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            rk->setDelta(thHuberMono);
            e->setRobustKernel(rk);
            e->fx = K(0, 0);
            e->fy = K(1, 1);
            e->cx = K(0, 2);
            e->cy = K(1, 2);
            optimizer.addEdge(e);

            vEdges.emplace_back(std::make_pair(e, std::vector<int>{chain.mStartIdx + i, (int)vPoints.size()-1, (int)vObs.size()-1}));

            // calc sum of square error before optimization
            Eigen::Vector2d diff = e->cam_project(vPoses[chain.mStartIdx + i]->estimate().map(vPoint->estimate())) - obs;
            sum_diff2 += diff.dot(diff);
            sum_chi2 += e->chi2();
        }
        vertexIdx++;
    }

    std::cout << "points square error before optimization: " << sum_diff2 / vEdges.size() << std::endl;
    std::cout << "chi2 before optimization: " << sum_chi2 / vEdges.size() << std::endl;

    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(40);

    sum_chi2 = 0;
    sum_diff2 = 0;
    for(auto& im: vEdges){
        sum_chi2 += im.first->chi2();
        auto* e = im.first;
        auto* pose = vPoses[im.second[0]];
        auto* pt = vPoints[im.second[1]];
        auto ob = vObs[im.second[2]];
        Eigen::Vector2d diff = e->cam_project(pose->estimate().map(pt->estimate())) - ob;
        sum_diff2 += diff.dot(diff);
    }
    std::cout << "points square error after optimization: " << sum_diff2 / vEdges.size() << std::endl;
    std::cout << "chi2 after optimization: " << sum_chi2 / vEdges.size() << std::endl;
    std::cout << "edge num=" << vEdges.size() << std::endl;
}

    
} // namespace Naive_SLAM_ROS
