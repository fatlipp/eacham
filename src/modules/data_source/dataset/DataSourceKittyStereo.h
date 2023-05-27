#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <istream>
#include <fstream>

#include "data_source/dataset/IDatasetVisual.h"

namespace eacham
{

template<typename T>
class DataSourceKittyStereo : public IDatasetVisual<T>
{
public:
    DataSourceKittyStereo(const std::string &sourcePath) 
        : IDatasetVisual<T>(CameraType::STEREO, sourcePath + "/data_odometry_poses/dataset/poses/00.txt")
        , id(0)
        , sourcePath(sourcePath)
        , folderLeft(sourcePath + "/data_odometry_gray/dataset/sequences/00/image_0/")
        , folderRight(sourcePath + "/data_odometry_gray/dataset/sequences/00/image_1/")
        , timestampFolder(sourcePath + "/data_odometry_gray/dataset/sequences/00/times.txt")
    {
    }

    ~DataSourceKittyStereo() override
    {
        if (timeFileStream.is_open())
        {
            timeFileStream.close();
        }
    }

public:
    void Initialize(const ConfigCamera& config) override;
    
    void Process() override;

private:
    const std::string sourcePath;
    const std::string folderLeft;
    const std::string folderRight;
    const std::string timestampFolder;
    mutable std::ifstream timeFileStream;

    mutable unsigned id;
};

}

namespace eacham
{

template<typename T>
void DataSourceKittyStereo<T>::Initialize(const ConfigCamera& config)
{
    this->timeFileStream.open(this->timestampFolder, std::ios::in);

    std::ifstream file;
    file.open(this->sourcePath + "/data_odometry_gray/dataset/sequences/00/calib.txt", std::ios::in);

    if (!file.is_open())
    {
        return;
    }

    std::string name;

    cv::Mat projMat = cv::Mat(3, 4, CV_32F);

    for (int i = 0; i < 4; ++i)
    {
        int count = 0;
        
        file >> name;

        while (count < 12)
        {
            float value;
            file >> value;

            if (name.find("P1:") == 0)
            {
                const int col = count % 4;
                const int row = count / 4;

                projMat.at<float>(row, col) = value;
            }

            ++count;
        }
    }

    // cv::Mat proj;
    // cv::Mat rot;
    // cv::Mat trans;
    // cv::decomposeProjectionMatrix(projMat, proj, rot, trans);
    // trans = trans / trans.at<float>(3);
    // std::cout << "proj:\n" << proj << std::endl;
    // std::cout << "rot:\n" << rot << std::endl;
    // std::cout << "trans:\n" << trans.t() << std::endl;

    auto cameraMat = cv::Mat(1, 5, CV_32F);
    cameraMat.at<float>(0, 0) = projMat.at<float>(0, 0);
    cameraMat.at<float>(0, 1) = projMat.at<float>(1, 1);
    cameraMat.at<float>(0, 2) = projMat.at<float>(0, 2);
    cameraMat.at<float>(0, 3) = projMat.at<float>(1, 2);
    cameraMat.at<float>(0, 4) = (-projMat.at<float>(0, 3));

    auto distMat = cv::Mat(1, 5, CV_32F);
    distMat.at<float>(0, 0) = 0;
    distMat.at<float>(0, 1) = 0;
    distMat.at<float>(0, 2) = 0;
    distMat.at<float>(0, 3) = 0;
    distMat.at<float>(0, 4) = 0;

    this->cameraMatrix = cameraMat;
    this->distMatrix = distMat;

    std::cout << "KITTI Stereo cameraMatrix:\n" << cameraMat << std::endl;
    std::cout << "KITTI Stereo distMatrix:\n" << distMat << std::endl;

    file.close();
}

// TODO: fix it
std::string format(const unsigned number)
{
    std::string prefix = "00000";

    if (number > 9)
    {
        prefix.pop_back();
        if (number > 99)
            prefix.pop_back();

        if (number > 999)
            prefix.pop_back();

        if (number > 9999)
            prefix.pop_back();

        if (number > 99999)
            prefix.pop_back();
    }

    return prefix + std::to_string(number);
}

template<typename T>
void DataSourceKittyStereo<T>::Process()
{
    if (this->dataUpdated)
    {
        return;
    }

    if (!this->timeFileStream.is_open() || !this->gtFileStream.is_open())
    {
        return;
    }

    double timestampCurrent = -1.0;
    timeFileStream >> timestampCurrent;

    cv::Mat imLeft = cv::imread(folderLeft  + format(this->id) + ".png");
    cv::Mat imRight = cv::imread(folderRight  + format(this->id) + ".png");

    Eigen::Matrix4f gtPos = Eigen::Matrix4f::Identity();
    this->gtFileStream >> gtPos(0, 0) >> gtPos(0, 1) >> gtPos(0, 2) >> gtPos(0, 3);
    this->gtFileStream >> gtPos(1, 0) >> gtPos(1, 1) >> gtPos(1, 2) >> gtPos(1, 3);
    this->gtFileStream >> gtPos(2, 0) >> gtPos(2, 1) >> gtPos(2, 2) >> gtPos(2, 3);

    std::lock_guard<std::mutex> lock(this->dataMutex);
    this->timestamp = timestampCurrent;
    this->imageLeft = imLeft.clone();
    this->imageRight = imRight.clone();
    this->groundTruthPos = gtPos;

    this->dataUpdated = true;

    if (!this->datasetUpdated)
    {
        this->timestampOld = this->timestamp;
        this->groundTruthPosOld = this->groundTruthPos;
    }

    ++this->id;
}

}