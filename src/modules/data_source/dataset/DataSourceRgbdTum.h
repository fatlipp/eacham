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
class DataSourceRgbdTum : public IDataSourceCamera<T>, public IDataset<T>
{
public:
    DataSourceRgbdTum(const std::string &sourcePath)
        : IDataset<T>(sourcePath + "/groundtruth.txt")
        , sourcePath(sourcePath + "/")
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

        this->cameraMatrix = cv::Mat(1, 5, CV_32F);
        this->cameraMatrix.at<float>(0, 0) = 535.4;
        this->cameraMatrix.at<float>(0, 1) = 539.2;
        this->cameraMatrix.at<float>(0, 2) = 320.1;
        this->cameraMatrix.at<float>(0, 3) = 247.6;
        this->cameraMatrix.at<float>(0, 4) = 1.0f / 5000.0;

        this->distortionMatrix = cv::Mat(1, 5, CV_32F);
        this->distortionMatrix.at<float>(0, 0) = 0;
        this->distortionMatrix.at<float>(0, 1) = 0;
        this->distortionMatrix.at<float>(0, 2) = 0;
        this->distortionMatrix.at<float>(0, 3) = 0;
        this->distortionMatrix.at<float>(0, 4) = 0;

        std::cout << "TUM cameraMatrix:\n" << this->cameraMatrix << std::endl;
        std::cout << "TUM distortionMatrix:\n" << this->distortionMatrix << std::endl;
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

    T Get() const override;

    void ReadNext() override;

    bool isStereo() const override
    {
        return false;
    }

    bool isRgbd() const override
    {
        return true;
    }

    cv::Mat GetParameters() const override;
    cv::Mat GetDistortion() const override;

private:
    const std::string sourcePath;

    mutable std::ifstream rgbFileStream;
    mutable std::ifstream depthFileStream;

    T currentData;

    cv::Mat_<float> cameraMatrix;
    cv::Mat_<float> distortionMatrix;

    mutable unsigned id;
};

}

namespace eacham
{

template<typename T>
void DataSourceRgbdTum<T>::ReadNext()
{
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

    auto imRgb = cv::imread(sourcePath + pathRgb);
    cv::cvtColor(imRgb, imRgb, cv::COLOR_BGR2GRAY);

    auto imDepth = cv::imread(sourcePath + pathDepth, cv::IMREAD_UNCHANGED);

    currentData = {timestampDepth, imRgb, imDepth};

    double timestampGt = -1.0;
    Eigen::Vector3f position;
    Eigen::Quaternionf quaternion;
    if (this->gtFileStream.is_open())
    {   
        // TODO: Shit code !!
        while (timestampGt < timestampDepth)
        {
            this->gtFileStream >> timestampGt;
            this->gtFileStream >> position.x() >> position.y() >> position.z();
            this->gtFileStream >> quaternion.x() >> quaternion.y() >> quaternion.z() >> quaternion.w();
        }
    }

    this->currentPose = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f matRot = quaternion.toRotationMatrix();
    this->currentPose.block(0, 0, 3, 3) = matRot;
    this->currentPose.block(0, 3, 3, 1) = position;

    ++id;
}

template<typename T>
T DataSourceRgbdTum<T>::Get() const
{
    return currentData;
}

template<typename T>
cv::Mat DataSourceRgbdTum<T>::GetParameters() const
{
    return this->cameraMatrix;
}

template<typename T>
cv::Mat DataSourceRgbdTum<T>::GetDistortion() const
{
    return this->distortionMatrix;
}



}