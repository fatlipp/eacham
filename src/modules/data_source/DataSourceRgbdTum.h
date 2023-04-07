#pragma once

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <algorithm>
#include <istream>
#include <fstream>

#include "IDataset.h"
#include "IDataSourceCamera.h"

namespace eacham
{

template<typename T>
class DataSourceRgbdTum : public IDataSourceCamera<T>, public IDataset<T>
{
public:
    DataSourceRgbdTum(const std::string &sourcePath)
        : sourcePath(sourcePath + "/")
    {
        const std::string folderRgb = (this->sourcePath + "rgb.txt");
        const std::string folderDepth = (this->sourcePath + "depth.txt");
        const std::string gtPoses = (this->sourcePath + "groundtruth.txt");

        rgbFileStream.open(folderRgb, std::ios::in);
        if (rgbFileStream.is_open())
        {
            while (rgbFileStream.peek() == '#')
            {
                std::string tmp;
                std::getline(rgbFileStream, tmp);
            }
        }
        depthFileStream.open(folderDepth, std::ios::in);
        if (depthFileStream.is_open())
        {
            while (depthFileStream.peek() == '#')
            {
                std::string tmp;
                std::getline(depthFileStream, tmp);
            }
        }
        gtFileStream.open(gtPoses, std::ios::in);
        if (gtFileStream.is_open())
        {
            while (gtFileStream.peek() == '#')
            {
                std::string tmp;
                std::getline(gtFileStream, tmp);
            }
        }
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

    ~DataSourceRgbdTum()
    {
        if (rgbFileStream.is_open())
        {
            rgbFileStream.close();
        }

        if (depthFileStream.is_open())
        {
            depthFileStream.close();
        }

        if (gtFileStream.is_open())
        {
            gtFileStream.close();
        }
    }

    T Get() const override;

    void ReadNext() override;
    Eigen::Matrix4f GetGtPose() const override;

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
    mutable std::ifstream gtFileStream;

    T currentData;
    Eigen::Matrix4f currentPose;

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

    // expl: 1341841278.842683 depth/1341841278.842683.png
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

    // if (imDepth.depth() != CV_8U)
    //     imDepth.convertTo(imDepth, CV_8U);
    std::cout << "imDepth: " << imDepth.type() << std::endl;
    currentData = {timestampDepth, imRgb, imDepth};

    double timestampGt = -1.0;
    Eigen::Vector3f position;
    Eigen::Quaternionf quaternion;
    if (gtFileStream.is_open())
    {   
        // TODO: Shit code !!
        while (timestampGt < timestampDepth)
        {
            gtFileStream >> timestampGt;
            gtFileStream >> position.x() >> position.y() >> position.z();
            gtFileStream >> quaternion.x() >> quaternion.y() >> quaternion.z() >> quaternion.w();
        }
    }

    currentPose = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f matRot = quaternion.toRotationMatrix();
    currentPose.block<3, 3>(0, 0) = matRot;
    currentPose.block<3, 1>(0, 3) = position;

    std::cout.precision(17);
    std::cout << "DATA: " << timestampRgb << ", " << timestampDepth << ", " << timestampGt << 
                 ", pos: " << position.transpose() << std::endl; 

    ++id;
}

template<typename T>
Eigen::Matrix4f DataSourceRgbdTum<T>::GetGtPose() const
{
    return currentPose;
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