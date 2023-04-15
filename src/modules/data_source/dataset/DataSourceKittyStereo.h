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
class DataSourceKittyStereo : public IDataSourceCamera<T>, public IDataset
{
public:
    DataSourceKittyStereo(const std::string &sourcePath) 
        : IDataSourceCamera<T>(CameraType::STEREO)
        , IDataset(sourcePath + "/data_odometry_poses/dataset/poses/00.txt")
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

    if (file.is_open())
    {
        std::string name;

        cv::Mat projMat = cv::Mat(3, 4, CV_32F);

        for (int i = 0; i < 4; ++i)
        {
            float value1;
            int count = 0;
            
            file >> name;

            while (count < 12)
            {
                file >> value1;

                const int col = count % 4;
                const int row = count / 4;

                if (name.find("P1:") == 0)
                {
                    projMat.at<float>(row, col) = value1;
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
    if (this->dataGot)
    {
        return;
    }

    double timestamp = -1.0;

    if (timeFileStream.is_open())
    {
        timeFileStream >> timestamp;
    }

    Eigen::Matrix4f redPos = Eigen::Matrix4f::Identity();

    if (this->gtFileStream.is_open())
    {
        this->gtFileStream >> redPos(0, 0) >> redPos(0, 1) >> redPos(0, 2) >> redPos(0, 3);
        this->gtFileStream >> redPos(1, 0) >> redPos(1, 1) >> redPos(1, 2) >> redPos(1, 3);
        this->gtFileStream >> redPos(2, 0) >> redPos(2, 1) >> redPos(2, 2) >> redPos(2, 3);
    }

    std::lock_guard<std::mutex> lock(this->dataMutex);
    this->imageLeft = cv::imread(folderLeft  + format(id) + ".png");
    this->imageRight = cv::imread(folderRight  + format(id) + ".png");
    this->groundTruthPos = redPos;
    this->dataGot = true;

    ++id;
}

}