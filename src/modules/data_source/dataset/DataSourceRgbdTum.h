#pragma once

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <algorithm>
#include <istream>
#include <fstream>

#include "data_source/dataset/IDatasetVisual.h"

namespace eacham
{

template<typename T>
class DataSourceRgbdTum : public IDatasetVisual<T>
{
public:
    DataSourceRgbdTum(const std::string &sourcePath)
        : IDatasetVisual<T>(CameraType::RGBD, sourcePath + "/groundtruth.txt")
        , sourcePath(sourcePath + "/")
    {
    }

    ~DataSourceRgbdTum() override
    {
        if (rgbFileStream.is_open())
        {
            rgbFileStream.close();
        }

        if (depthFileStream.is_open())
        {
            depthFileStream.close();
        }
    }

public:
    void Initialize(const ConfigCamera& config) override;

    void Process() override;

private:
    const std::string sourcePath;

    mutable std::ifstream rgbFileStream;
    mutable std::ifstream depthFileStream;
};

}

namespace eacham
{

void IgnoreCaption(std::ifstream& stream)
{
    if (!stream.is_open())
    {
        return;
    }

    while (stream.peek() == '#')
    {
        std::string tmp;
        std::getline(stream, tmp);
    }
}

template<typename T>
void DataSourceRgbdTum<T>::Initialize(const ConfigCamera& config)
{
    const std::string folderRgb = (this->sourcePath + "rgb.txt");
    const std::string folderDepth = (this->sourcePath + "depth.txt");

    this->rgbFileStream.open(folderRgb, std::ios::in);
    this->depthFileStream.open(folderDepth, std::ios::in);

    IgnoreCaption(this->rgbFileStream);
    IgnoreCaption(this->depthFileStream);
    IgnoreCaption(this->gtFileStream);

    std::cout << "sourcePath: " << this->sourcePath << std::endl;
    std::cout << "folderRgb: " << folderRgb << std::endl;
    std::cout << "folderDepth: " << folderDepth << std::endl;

    // Camera.width: 640
    // Camera.height: 480

    // 1
    // Camera1.fx: 517.306408
    // Camera1.fy: 516.469215
    // Camera1.cx: 318.643040
    // Camera1.cy: 255.313989
    // Camera1.k1: 0.262383
    // Camera1.k2: -0.953104
    // Camera1.k3: -0.005358
    // Camera1.k4: 0.002628
    // Camera1.k5: 1.163314

    // 2
    // Camera1.fx: 520.908620
    // Camera1.fy: 521.007327
    // Camera1.cx: 325.141442
    // Camera1.cy: 249.701764
    // Camera1.k1: 0.231222
    // Camera1.k2: -0.784899
    // Camera1.k3: -0.003257
    // Camera1.k4: -0.000105
    // Camera1.k5: 0.917205

    // 3
    // Camera1.fx: 535.4
    // Camera1.fy: 539.2
    // Camera1.cx: 320.1
    // Camera1.cy: 247.6
    // Camera1.k1: 0.0
    // Camera1.k2: 0.0
    // Camera1.k3: 0.0
    // Camera1.k4: 0.0

    auto cameraMat = cv::Mat(1, 5, CV_32F);
    cameraMat.at<float>(0, 0) = 535.4;
    cameraMat.at<float>(0, 1) = 539.2;
    cameraMat.at<float>(0, 2) = 320.1;
    cameraMat.at<float>(0, 3) = 247.6;
    cameraMat.at<float>(0, 4) = 1.0f / 5000.0;

    auto distMat = cv::Mat(1, 5, CV_32F);
    distMat.at<float>(0, 0) = 0;
    distMat.at<float>(0, 1) = 0;
    distMat.at<float>(0, 2) = 0;
    distMat.at<float>(0, 3) = 0;
    distMat.at<float>(0, 4) = 0;

    this->cameraMatrix = cameraMat;
    this->distMatrix = distMat;

    std::cout << "TUM cameraMatrix:\n" << cameraMat << std::endl;
    std::cout << "TUM distMatrix:\n" << distMat << std::endl;
}

template<typename T>
void DataSourceRgbdTum<T>::Process()
{
    if (this->dataUpdated)
    {
        return;
    }

    if (!this->rgbFileStream.is_open() || !this->depthFileStream.is_open() || !this->gtFileStream.is_open())
    {
        return;
    }
    
    // read data
    double timestamp = -1.0;
    std::string pathRgb;
    rgbFileStream >> timestamp;
    rgbFileStream >> pathRgb;

    std::string pathDepth;
    depthFileStream >> timestamp;
    depthFileStream >> pathDepth;

    auto imRgb = cv::imread(sourcePath + pathRgb);
    cv::cvtColor(imRgb, imRgb, cv::COLOR_BGR2GRAY);
    auto imDepth = cv::imread(sourcePath + pathDepth, cv::IMREAD_UNCHANGED);
    
    Eigen::Vector3f position;
    Eigen::Quaternionf quaternion;

    // TODO: Shit code !!
    double timestampGt = -1.0;
    while (timestampGt < timestamp)
    {
        this->gtFileStream >> timestampGt;
        this->gtFileStream >> position.x() >> position.y() >> position.z();
        this->gtFileStream >> quaternion.x() >> quaternion.y() >> quaternion.z() >> quaternion.w();
    }

    // update data
    std::lock_guard<std::mutex> lock(this->dataMutex);

    this->imageLeft = imRgb;
    this->imageRight = imDepth;
    this->timestamp = timestamp;

    this->groundTruthPos = Eigen::Matrix4f::Identity();
    this->groundTruthPos.block(0, 0, 3, 3) = quaternion.toRotationMatrix();
    this->groundTruthPos.block(0, 3, 3, 1) = position;

    {
        static Eigen::Matrix4f defaultPosInv = this->groundTruthPos.inverse();
        this->groundTruthPos = defaultPosInv * this->groundTruthPos;
    }

    if (!this->datasetUpdated)
    {
        this->timestampOld = this->timestamp;
        this->groundTruthPosOld = this->groundTruthPos;
    }

    this->dataUpdated = true;
}

}