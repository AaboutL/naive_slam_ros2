//
// Created by hanfuyong on 2022/10/13.
//

#include "ImageData.h"

namespace Naive_SLAM_ROS {
ImageData::ImageData(const ImageData& imageData):
    N(imageData.N), mImg(imageData.mImg.clone()),
    mK(imageData.mK.clone()), mDistCoef(imageData.mDistCoef.clone()),
    mImgWidth(imageData.mImgWidth), mImgHeight(imageData.mImgHeight),
    mCellSize(imageData.mCellSize),
    mGridRows(imageData.mGridRows), mGridCols(imageData.mGridCols),
    mvKPs(imageData.mvKPs), mvKPsUn(imageData.mvKPsUn),
    mvPts(imageData.mvPts), mvPtsUn(imageData.mvPtsUn),
    mvPtUnOffsets(imageData.mvPtUnOffsets),
    mDescriptors(imageData.mDescriptors.clone()),
    mvChainIds(imageData.mvChainIds), mvChainLens(imageData.mvChainLens){
}

ImageData::ImageData(const cv::Mat &img,
             const cv::Mat& K, const cv::Mat& distCoef,
             int imgWidth, int imgHeight, int cellSize, int gridRows, int gridCols,
             std::shared_ptr<ORBextractor> pORBextractor) :
        mImg(img.clone()), mK(K.clone()), mDistCoef(distCoef.clone()),
        mImgWidth(imgWidth), mImgHeight(imgHeight), mCellSize(cellSize), 
        mGridRows(gridRows), mGridCols(gridCols) {

    // ExtractORB(img);
    (*pORBextractor)(img, cv::Mat(), mvKPs, mDescriptors);
    
    N = mvKPs.size();
    UndistortKeyPoints();

    mvPts.resize(N);
    mvPtsUn.resize(N);
    mvPtUnOffsets.resize(N);
    mvChainIds.resize(N);
    mvChainLens.resize(N);
    for (size_t i = 0; i < N; i++) {
        mvPts[i] = mvKPs[i].pt;
        mvPtsUn[i] = mvKPsUn[i].pt;
        mvPtUnOffsets[i] = cv::Point2f(0, 0);
        mvChainIds[i] = -1;
        mvChainLens[i] = 1;
    }

    mGrid = new std::vector<size_t> *[mGridRows];
    for (int i = 0; i < mGridRows; i++) {
        mGrid[i] = new std::vector<size_t>[mGridCols];
    }

    AssignGrid();
}

ImageData::ImageData(int featureNum, const cv::Mat &img, const cv::Mat& K, const cv::Mat& distCoef,
              int imgWidth, int imgHeight):
        mImg(img.clone()), mK(K.clone()), mDistCoef(distCoef.clone()),
        mImgWidth(imgWidth), mImgHeight(imgHeight), N(featureNum){
    mvChainIds.resize(N, -1);
    mvChainLens.resize(N, 1);
    mvPtUnOffsets.resize(N, cv::Point2f(0, 0));
}

// void ImageData::ExtractORB(const cv::Mat &img) {
//     (*mpORBextractor)(img, cv::Mat(), mvKPs, mDescriptors);
// }

void ImageData::UndistortKeyPoints() {
    if (mDistCoef.at<float>(0) == 0.0) {
        mvKPsUn = mvKPs;
        return;
    }

    cv::Mat mat(N, 2, CV_32F);
    for (int i = 0; i < N; i++) {
        mat.at<float>(i, 0) = mvKPs[i].pt.x;
        mat.at<float>(i, 1) = mvKPs[i].pt.y;
    }

    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
    mat = mat.reshape(1);

    mvKPsUn.resize(N);
    for (int i = 0; i < N; i++) {
        cv::KeyPoint kp = mvKPs[i];
        kp.pt.x = mat.at<float>(i, 0);
        kp.pt.y = mat.at<float>(i, 1);
        mvKPsUn[i] = kp;
    }
}

void ImageData::UndistortKeyPoints1() {
    if (mvPts.empty()) {
        std::cout << "No points to undistorted!" << std::endl;
        exit(0);
    }

    cv::Mat mat(mvPts.size(), 2, CV_32F);
    for (int i = 0; i < mvPts.size(); i++) {
        mat.at<float>(i, 0) = mvPts[i].x;
        mat.at<float>(i, 1) = mvPts[i].y;
    }

    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
    mat = mat.reshape(1);

    mvPtsUn.resize(mvPts.size(), cv::Point2f(0, 0));
    for (int i = 0; i < mvPts.size(); i++) {
        cv::Point2f ptUn(mat.at<float>(i, 0), mat.at<float>(i, 1));
        mvPtsUn[i] = ptUn;
    }
}

void ImageData::ExtractHarris(){
    int existPtsNum = mvPts.size();
    int needNum = N - existPtsNum;
    if(needNum == 0) 
        return;
    cv::Mat mask;
    if(needNum != N){
        mask = cv::Mat(mImgHeight, mImgWidth, CV_8UC1, cv::Scalar(255));
        for(auto& pt: mvPts){
            if(mask.at<uchar>(pt) == 255){
                cv::circle(mask, pt, 30, 0, -1);
            }
        }
    }
    std::vector<cv::Point2f> vPts;
    cv::goodFeaturesToTrack(mImg, vPts, needNum, 0.01, 30, mask);
    if(needNum == N){
        mvPts = vPts;
    }
    else{
        for(auto& pt : vPts){
            mvPts.emplace_back(pt);
            mvPtUnOffsets.emplace_back(cv::Point2f(0, 0));
            mvChainIds.emplace_back(-1);
            mvChainLens.emplace_back(1);
        }
    }
}

/*
 * ???????????????????????????????????????grid???????????????????????????????????????????????????grid
 */
void ImageData::AssignGrid() {
    for (int i = 0; i < N; i++) {
        cv::Point2f ptUn = mvPtsUn[i];
        if (ptUn.x < 0 || ptUn.x >= mImgWidth || ptUn.y < 0 || ptUn.y >= mImgHeight)
            continue;
        int colIdx = int(ptUn.x / mCellSize);
        int rowIdx = int(ptUn.y / mCellSize);
        mGrid[rowIdx][colIdx].emplace_back(i);
    }
}

int ImageData::SearchGrid(const cv::Point2f &pt2d, const cv::Mat &description,
                           float radius, int &matchedId){
    int nMinCellX = std::max(0, (int) std::floor(pt2d.x - radius) / mCellSize);
    int nMinCellY = std::max(0, (int) std::floor(pt2d.y - radius) / mCellSize);
    int nMaxCellX = std::min(mGridCols - 1,
                             (int) std::ceil(pt2d.x + radius) / mCellSize);
    int nMaxCellY = std::min(mGridRows - 1,
                             (int) std::ceil(pt2d.y + radius) / mCellSize);

    int bestDist = INT_MAX;
    int bestId = -1;
    for (int ci = nMinCellY; ci <= nMaxCellY; ci++) {
        for (int cj = nMinCellX; cj <= nMaxCellX; cj++) {
            std::vector<std::size_t> candidatePtsIdx = mGrid[ci][cj];
            for (auto j: candidatePtsIdx) {
                cv::KeyPoint kpUn = mvKPsUn[j];
                if (fabs(kpUn.pt.x - pt2d.x) > radius || fabs(kpUn.pt.y - pt2d.y) > radius) {
                    continue;
                }
                cv::Mat curDesp = mDescriptors.row(j);
                int dist = DescriptorDistance(description, curDesp);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestId = j;
                }
            }
        }
    }
    matchedId = bestId;
    return bestDist;

}

int ImageData::DescriptorDistance(const cv::Mat &a, const cv::Mat &b) {
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist = 0;

    for (int i = 0; i < 8; i++, pa++, pb++) {
        unsigned int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

} //namespace naive_slam_ros
