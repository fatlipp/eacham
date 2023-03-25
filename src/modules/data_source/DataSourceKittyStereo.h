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
class DataSourceKittyStereo : public IDataSourceCamera<T>, public IDataset<T>
{
public:
    DataSourceKittyStereo(const std::string &sourcePath) 
        : folderLeft(sourcePath + "/data_odometry_gray/dataset/sequences/00/image_0/")
        , folderRight(sourcePath + "/data_odometry_gray/dataset/sequences/00/image_1/")
        , timestampFolder(sourcePath + "/data_odometry_gray/dataset/sequences/00/times.txt")
        , gtPoses(sourcePath + "/data_odometry_poses/dataset/poses/00.txt")
    {
        timeFileStream.open(timestampFolder, std::ios::in);
        gtFileStream.open(gtPoses, std::ios::in);

        std::ifstream file;
        file.open(sourcePath + "/data_odometry_gray/dataset/sequences/00/calib.txt", std::ios::in);

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

            this->cameraMatrix = cv::Mat(1, 5, CV_32F);
            this->cameraMatrix.at<float>(0, 0) = projMat.at<float>(0, 0);
            this->cameraMatrix.at<float>(0, 1) = projMat.at<float>(1, 1);
            this->cameraMatrix.at<float>(0, 2) = projMat.at<float>(0, 2);
            this->cameraMatrix.at<float>(0, 3) = projMat.at<float>(1, 2);
            this->cameraMatrix.at<float>(0, 4) = (-projMat.at<float>(0, 3));
            // this->cameraMatrix.at<float>(0, 4) = (trans.at<float>(0));

            std::cout << "KITTI Stereo cameraMatrix:\n" << this->cameraMatrix << std::endl;

            file.close();
        }
    }

    ~DataSourceKittyStereo()
    {
        if (timeFileStream.is_open())
        {
            timeFileStream.close();
        }

        if (gtFileStream.is_open())
        {
            gtFileStream.close();
        }
    }

    T Get() const override;

    void ReadNext() override;
    Eigen::Matrix4f GetGtPose() const override;

    cv::Mat GetParameters() const override;
    cv::Mat GetDistortion() const override;

private:
    const std::string folderLeft;
    const std::string folderRight;
    const std::string timestampFolder;
    const std::string gtPoses;
    mutable std::ifstream timeFileStream;
    mutable std::ifstream gtFileStream;

    T currentData;
    Eigen::Matrix4f currentPose;

    cv::Mat_<float> cameraMatrix;

    mutable unsigned id;
};

}

namespace eacham
{

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
void DataSourceKittyStereo<T>::ReadNext()
{
    const auto im1 = cv::imread(folderLeft  + format(id) + ".png");
    const auto im2 = cv::imread(folderRight + format(id) + ".png");
    double timestamp = -1.0;

    if (timeFileStream.is_open())
    {
        timeFileStream >> timestamp;
    }

    currentData = {timestamp, im1.clone(), im2.clone()};
    
    currentPose = Eigen::Matrix4f::Identity();

    if (gtFileStream.is_open())
    {
        gtFileStream >> currentPose(0, 0) >> currentPose(0, 1) >> currentPose(0, 2) >> currentPose(0, 3);
        gtFileStream >> currentPose(1, 0) >> currentPose(1, 1) >> currentPose(1, 2) >> currentPose(1, 3);
        gtFileStream >> currentPose(2, 0) >> currentPose(2, 1) >> currentPose(2, 2) >> currentPose(2, 3);
    }

    ++id;
}

template<typename T>
Eigen::Matrix4f DataSourceKittyStereo<T>::GetGtPose() const
{
    return currentPose;
}

template<typename T>
T DataSourceKittyStereo<T>::Get() const
{
    return currentData;
}

template<typename T>
cv::Mat DataSourceKittyStereo<T>::GetParameters() const
{
    return cameraMatrix;
}

template<typename T>
cv::Mat DataSourceKittyStereo<T>::GetDistortion() const
{
    return cv::Mat::zeros(1, 5, CV_32F);
}



}