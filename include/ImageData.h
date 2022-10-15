//
// Created by hanfuyong on 2022/10/13.
//

#ifndef NAIVE_SLAM_ROS_IMAGEDATA_H
#define NAIVE_SLAM_ROS_IMAGEDATA_H

#include <opencv2/opencv.hpp>
#include "ORBextractor.h"

namespace Naive_SLAM_ROS {

class ImageData{
public:
    ImageData(){}
    ImageData(const ImageData& imageData);
    ImageData(const cv::Mat &img,
              const cv::Mat& K, const cv::Mat& distCoef,
              int imgWidth, int imgHeight, int cellSize, int gridRows, int gridCols,
              std::shared_ptr<ORBextractor> extractor);

    int SearchGrid(const cv::Point2f &pt2d, const cv::Mat &description,
                    float radius, int &matchedId);

private:
    void ExtractORB(const cv::Mat &img);
    void UndistortKeyPoints();
    void AssignGrid();
    int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);


public:
    int N;

    cv::Mat mImg;

    cv::Mat mK;
    cv::Mat mDistCoef;
    int mImgWidth;
    int mImgHeight;
    int mCellSize;
    int mGridRows;
    int mGridCols;
    std::vector<std::size_t>** mGrid;

    std::vector<cv::KeyPoint> mvKPs;
    std::vector<cv::KeyPoint> mvKPsUn;
    std::vector<cv::Point2f> mvPts;
    std::vector<cv::Point2f> mvPtsUn;
    cv::Mat mDescriptors;
    std::vector<long unsigned int> mvChainIds;
    std::vector<int> mvChainLens;

    std::shared_ptr<ORBextractor> mpORBextractor;
};

}

#endif //NAIVE_SLAM_ROS_IMAGEDATA_H
