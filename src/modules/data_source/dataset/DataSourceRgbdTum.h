#pragma once

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <algorithm>
#include <istream>
#include <fstream>

#include "data_source/IDataSourceCamera.h"
#include "data_source/dataset/IDataset.h"

namespace eacham
{

template<typename T>
class DataSourceRgbdTum : public IDataSourceCamera<T>, public IDataset
{
public:
    DataSourceRgbdTum(const std::string &sourcePath)
        : IDataSourceCamera<T>(CameraType::RGBD)
        , IDataset(sourcePath + "/groundtruth.txt")
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

public:
    Eigen::Matrix4f GetGtPose() const override
    {
        while (!this->dataGot)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
        
        return groundTruthPos;
    }

private:
    const std::string sourcePath;

    mutable std::ifstream rgbFileStream;
    mutable std::ifstream depthFileStream;

    T currentData;

    mutable unsigned id;
};

}

namespace eacham
{

template<typename T>
void DataSourceRgbdTum<T>::Initialize(const ConfigCamera& config)
{
    const std::string folderRgb = (this->sourcePath + "rgb.txt");
    const std::string folderDepth = (this->sourcePath + "depth.txt");

    this->rgbFileStream.open(folderRgb, std::ios::in);
    this->depthFileStream.open(folderDepth, std::ios::in);

    while (this->rgbFileStream.peek() == '#')
    {
        std::string tmp;
        std::getline(this->rgbFileStream, tmp);
    }

    while (this->depthFileStream.peek() == '#')
    {
        std::string tmp;
        std::getline(this->depthFileStream, tmp);
    }

    while (this->gtFileStream.peek() == '#')
    {
        std::string tmp;
        std::getline(this->gtFileStream, tmp);
    }

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
    if (this->dataGot)
    {
        return;
    }
    
    double timestampRgb = -1.0;
    std::string pathRgb;

    if (rgbFileStream.is_open())
    {
        rgbFileStream >> timestampRgb;
        rgbFileStream >> pathRgb;
    }

    double timestampDepth = -1.0;
    std::string pathDepth;
    if (depthFileStream.is_open())
    {
        depthFileStream >> timestampDepth;
        depthFileStream >> pathDepth;
    }

    std::lock_guard<std::mutex> lock(this->dataMutex);
    auto imRgb = cv::imread(sourcePath + pathRgb);
    cv::cvtColor(imRgb, imRgb, cv::COLOR_BGR2GRAY);

    this->imageLeft = imRgb;
    this->imageRight = cv::imread(sourcePath + pathDepth, cv::IMREAD_UNCHANGED);
    this->groundTruthPos = Eigen::Matrix4f::Identity();

    if (this->gtFileStream.is_open())
    {   
        double timestampGt = -1.0;
        Eigen::Vector3f position;
        Eigen::Quaternionf quaternion;

        // TODO: Shit code !!
        while (timestampGt < timestampDepth)
        {
            this->gtFileStream >> timestampGt;
            this->gtFileStream >> position.x() >> position.y() >> position.z();
            this->gtFileStream >> quaternion.x() >> quaternion.y() >> quaternion.z() >> quaternion.w();
        }

        Eigen::Matrix3f matRot = quaternion.toRotationMatrix();
        this->groundTruthPos.block(0, 0, 3, 3) = matRot;
        this->groundTruthPos.block(0, 3, 3, 1) = position;
    }

    this->dataGot = true;

    ++id;
}

}