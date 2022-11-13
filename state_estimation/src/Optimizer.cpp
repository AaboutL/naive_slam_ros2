//
// Created by hanfuyong on 2022/10/22
//

#include "Optimizer.h"
#include "utils.h"

namespace Naive_SLAM_ROS{

int Optimizer::VisualOnlyInitBAInvertDepth(Frame& frame1, Frame& frame2, std::shared_ptr<FeatureManager> pFM,
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
        pFM->SetWorldPos(chainId, vPoint->estimate());
        pFM->SetChainGood(chainId, true);
    }
}

int Optimizer::VisualOnlyInitBA(Frame* frame1, Frame* frame2, 
                        FeatureManager* pFM, std::vector<Eigen::Vector3d>& vPts3D,
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
    vSE31->setEstimate(g2o::SE3Quat(frame1->mRcw, frame1->mtcw));
    vSE31->setId(vertexIdx++);
    vSE31->setFixed(true);
    optimizer.addVertex(vSE31);

    auto *vSE32 = new g2o::VertexSE3Expmap();
    vSE32->setEstimate(g2o::SE3Quat(frame2->mRcw, frame2->mtcw));
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
    std::cout << "[Optimizer::VisualOnlyInitBA] points square error before optimization: " << sum_diff2 / vVid2Cid.size() << std::endl;
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
    std::cout << "[Optimizer::VisualOnlyInitBA] points square error after optimization: " << sum_diff2 / vVid2Cid.size() << std::endl;

    frame1->SetTcw(vSE31->estimate().to_homogeneous_matrix());
    frame2->SetTcw(vSE32->estimate().to_homogeneous_matrix());

    for(auto& p : vVid2Cid){
        auto vertexId = p.first;
        auto chainId = p.second;
        auto *vPoint = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(vertexId));
        pFM->SetWorldPos(chainId, vPoint->estimate());
        pFM->SetChainGood(chainId, true);
    }
    std::cout << "[Optimizer::VisualOnlyInitBA] finished" << std::endl;
    return 1;
}


int Optimizer::VisualOnlyBA(std::vector<Frame*>& vpFrames, FeatureManager* pFM, const Eigen::Matrix3d& K){
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
    for(int i = 0; i < vpFrames.size(); i++){
        auto *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(g2o::SE3Quat(vpFrames[i]->mRcw, vpFrames[i]->mtcw));
        vSE3->setFixed(vertexIdx == 0);
        vSE3->setId(vertexIdx++);
        optimizer.addVertex(vSE3);
        vPoses.emplace_back(vSE3);
    }

    // add 3D point vertex
    int edge_num = 0;
    int used_chain = 0;
    double sum_chi2 = 0;
    // std::vector<std::pair<g2o::EdgeSE3ProjectXYZ*, std::vector<int>>> vEdges;
    std::vector<std::pair<g2o::VertexPointXYZ*, unsigned long>> vPoints;
    std::vector<std::vector<Eigen::Vector2d>> vvObs;
    std::vector<std::vector<g2o::EdgeSE3ProjectXYZ*>> vvEdges;
    for(auto& [id, chain] : pFM->GetChains()){
        if(!chain.mbPosSet)
            continue;
        used_chain++;
        auto *vPoint = new g2o::VertexPointXYZ();
        vPoint->setEstimate(chain.mWorldPos);
        vPoint->setId(vertexIdx);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        vPoints.emplace_back(std::make_pair(vPoint, id));

        std::vector<Eigen::Vector2d> vObs;
        std::vector<g2o::EdgeSE3ProjectXYZ*> vEdges;
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

            vEdges.emplace_back(e);
            vObs.emplace_back(obs);
            edge_num++;

            // calc sum of square error before optimization
            // Eigen::Vector2d diff = e->cam_project(vPoses[chain.mStartIdx + i]->estimate().map(vPoint->estimate())) - obs;
            // sum_diff2 += diff.dot(diff);
            sum_chi2 += e->chi2();
        }
        vvEdges.emplace_back(vEdges);
        vvObs.emplace_back(vObs);
        vertexIdx++;
    }

    std::cout << "chi2 before optimization: " << sum_chi2 / edge_num << std::endl;

    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(20);

    for(int i = 0; i < vpFrames.size(); i++){
        vpFrames[i]->SetTcw(vPoses[i]->estimate().to_homogeneous_matrix());
    }

    sum_chi2 = 0;
    double sum_chi2_good = 0;
    int goodChainNum = 0;
    int goodEdgeNum = 0;
    for(int i = 0; i < vPoints.size(); i++){
        bool chainGood = true;
        double chain_chi2 = 0;
        int edgeNum = 0;
        for(int j = 0; j < vvEdges[i].size(); j++){
            double chi2 = vvEdges[i][j]->chi2();
            chain_chi2 += chi2;
            sum_chi2 += chi2;
            edgeNum++;
            if (chi2 > 5.991){
                chainGood = false;
            }
        }
        if(chainGood){
            sum_chi2_good += chain_chi2;
            goodChainNum++;
            goodEdgeNum += edgeNum;
            pFM->UpdateWorldPos(vPoints[i].second, vPoints[i].first->estimate());
            pFM->SetChainGood(vPoints[i].second, true);
        }
    }
    std::cout << "chi2 after optimization: " << sum_chi2 / edge_num << std::endl;
    std::cout << "chi2 after optimization good: " << sum_chi2_good / goodEdgeNum << std::endl;
    std::cout << "edge_num=" << edge_num << std::endl;
    std::cout << "edge_num good=" << goodEdgeNum << std::endl;
    std::cout << "good chain num=" << goodChainNum << std::endl;
    std::cout << "used chain num=" << used_chain << std::endl;
    std::cout << "total chain num=" << pFM->GetChains().size() << std::endl;
    return goodChainNum;
}

int Optimizer::VIInitOptimize(std::vector<Frame*>& vpFrames, const Eigen::Matrix3d& Rwg, double scale, double priorAcc, double priorGyr){
    std::cout << "[Optimizer::VIInitOptimize] Start" << std::endl;
    g2o::SparseOptimizer optimizer;
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver =
            std::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr =
            std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver));
    auto *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    optimizer.setAlgorithm(solver);

    int usedFrameNum = vpFrames.size() - 1;
    for(int i = 0; i < usedFrameNum; i++){
        auto* pF = vpFrames[i];

        VertexPose* vPose = new VertexPose(pF->GetTbc(), pF->GetTcw());
        vPose->setId(i);
        vPose->setFixed(true);
        optimizer.addVertex(vPose);

        VertexVelocity* vVel = new VertexVelocity(pF->GetVelocity());
        vVel->setId(i + usedFrameNum);
        vVel->setFixed(false);
        optimizer.addVertex(vVel);
    }
    // bias
    VertexGyrBias* vGB = new VertexGyrBias(vpFrames.back()->GetGyrBias());
    vGB->setId(usedFrameNum * 2);
    vGB->setFixed(false);
    optimizer.addVertex(vGB);

    VertexAccBias* vAB = new VertexAccBias(vpFrames.back()->GetAccBias());
    vAB->setId(usedFrameNum * 2 + 1);
    vAB->setFixed(false);
    optimizer.addVertex(vAB);

    // Gdir
    VertexGDir* vGdir = new VertexGDir(Rwg);
    vGdir->setId(usedFrameNum * 2 + 2);
    vGdir->setFixed(false);
    optimizer.addVertex(vGdir);
    
    // scale
    VertexScale* vS = new VertexScale(scale);
    vS->setId(usedFrameNum * 2 + 3);
    vS->setFixed(false);
    optimizer.addVertex(vS);

    // prior edge
    EdgePriorAccBias* ePAB = new EdgePriorAccBias(Eigen::Vector3d(0, 0, 0));
    ePAB->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vAB));
    ePAB->setInformation(Eigen::Matrix3d::Identity() * priorAcc);
    optimizer.addEdge(ePAB);

    EdgePriorGyrBias* ePGB = new EdgePriorGyrBias(Eigen::Vector3d(0, 0, 0));
    ePGB->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vAB));
    ePGB->setInformation(Eigen::Matrix3d::Identity() * priorGyr);
    optimizer.addEdge(ePGB);

    for(int i = 0; i < usedFrameNum - 1; i++){
        auto* pF = vpFrames[i];

        auto* vPose1 = optimizer.vertex(i);
        auto* vVel1 = optimizer.vertex(i + usedFrameNum);
        auto* vPose2 = optimizer.vertex(i + 1);
        auto* vVel2 = optimizer.vertex(i + 1 + usedFrameNum);
        auto* vGB = optimizer.vertex(usedFrameNum * 2);
        auto* vAB = optimizer.vertex(usedFrameNum * 2 + 1);
        auto* vGdir = optimizer.vertex(usedFrameNum * 2 + 2);
        auto* vS = optimizer.vertex(usedFrameNum * 2 + 3);

        auto* eInertialGS = new EdgeInertialGS(pF->mpPreintegrator);
        eInertialGS->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vPose1));
        eInertialGS->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vVel1));
        eInertialGS->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vGB));
        eInertialGS->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vAB));
        eInertialGS->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vPose2));
        eInertialGS->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vVel2));
        eInertialGS->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vGdir));
        eInertialGS->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vS));
        optimizer.addEdge(eInertialGS);
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(200);

    std::cout << "[Optimizer::VIInitOptimize] Done" << std::endl;
    return 1;
}
    
} // namespace Naive_SLAM_ROS
