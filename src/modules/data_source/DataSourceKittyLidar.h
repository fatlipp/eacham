#pragma once

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <algorithm>
#include <istream>
#include <fstream>

#include "IDataset.h"
#include "IDataSourceLidar.h"

namespace eacham
{

template<typename T>
class DataSourceKittyLidar : public IDataSourceLidar<T>, public IDataset<T>
{
public:
    DataSourceKittyLidar(const std::string &sourcePath) 
        : folder(sourcePath + "/data_odometry_velodyne/dataset/sequences/00/velodyne/")
        , id(0)
    {
        Eigen::Matrix4f lidarFrame;
        std::ifstream file;
        file.open(sourcePath + "/data_odometry_calib/dataset/sequences/00/calib.txt", std::ios::in);

        this->calibrationMatrix = Eigen::Matrix4f::Identity();

        if (file.is_open())
        {
            std::string name;

            cv::Mat projMat = cv::Mat(3, 4, CV_32F);

            for (int i = 0; i < 5; ++i)
            {
                float value1;
                int count = 0;
                
                file >> name;

                while (count < 12)
                {
                    file >> value1;

                    const int col = count % 4;
                    const int row = count / 4;

                    if (name.find("Tr:") == 0)
                    {
                        this->calibrationMatrix(row, col) = value1;
                    }

                    ++count;
                }
            }

            std::cout << "Lidar CalibrationMatrix:\n" << this->calibrationMatrix << std::endl;

            file.close();
        }
    }

    ~DataSourceKittyLidar()
    {
    }

    T Get() const override;

    void ReadNext() override;
    
    Eigen::Matrix4f GetGtPose() const override
    {
        return Eigen::Matrix4f::Identity();
    }

private:
    const std::string folder;
    Eigen::Matrix4f calibrationMatrix;
    T currentData;

    mutable unsigned id;
};

}

namespace eacham
{

// TODO: fix it
std::string format2(const unsigned number)
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
void DataSourceKittyLidar<T>::ReadNext()
{
    const auto fileName = this->folder + format2(id) + ".bin";

    std::ifstream file(fileName, std::ios::in | std::ios::binary);

    currentData.clear();
    
    Eigen::Vector4f point;
    int count = 0;
    float item;

    while (file.read((char*)&item, 4))
    {
        count++;
        if (count == 1)
        {
            point.x() = item;
        }
        else if (count == 2)
        {
            point.y() = item;
        }
        else if (count == 3)
        {
            point.z() = item;
        }
        else if (count == 4)
        {
            // point.w() = item;
            count = 0;
            point.w() = 1.0f;
            point = this->calibrationMatrix * point;

            currentData.push_back(Eigen::Vector3f { point.x(), point.y(), point.z() } );
        }
    }

    std::cout << "currentData = " << currentData.size() << ", fileName: " << fileName << std::endl;

    ++id;
}

template<typename T>
T DataSourceKittyLidar<T>::Get() const
{
    return currentData;
}

}