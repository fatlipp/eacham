#pragma once

#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <algorithm>
#include <istream>

#include "IDataSourceCamera.h"

namespace data_source
{

template<typename T>
class DataSourceKittyStereo : public IDataSourceCamera<T>
{
public:
    DataSourceKittyStereo(const std::string &sourcePath) 
        : folderLeft(sourcePath + "/dataset/sequences/00/image_0/")
        , folderRight(sourcePath + "/dataset/sequences/00/image_1/")
        , timestampFolder(sourcePath + "/dataset/sequences/00/times.txt")
    {
        timeFileStream.open(timestampFolder, std::ios::in);

        std::ifstream file;
        file.open(sourcePath + "/dataset/sequences/00/calib.txt", std::ios::in);

        if (file.is_open())
        {
            // this->cameraMatrix

            std::cout << "matrix: \n";

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
                        // this->camera1Matrix(row, col) = value1;
                        projMat.at<float>(row, col) = value1;
                    }

                    ++count;
                }
            }

            cv::Mat proj;
            cv::Mat rot;
            cv::Mat trans;
            cv::decomposeProjectionMatrix(projMat, proj, rot, trans);
            trans = trans / trans.at<float>(3);
            std::cout << "proj:\n" << proj << std::endl;
            std::cout << "rot:\n" << rot << std::endl;
            std::cout << "trans:\n" << trans.t() << std::endl;

            this->cameraMatrix = cv::Mat(1, 5, CV_32F);
            this->cameraMatrix.at<float>(0, 0) = projMat.at<float>(0, 0);
            this->cameraMatrix.at<float>(0, 1) = projMat.at<float>(1, 1);
            this->cameraMatrix.at<float>(0, 2) = projMat.at<float>(0, 2);
            this->cameraMatrix.at<float>(0, 3) = projMat.at<float>(1, 2);
            this->cameraMatrix.at<float>(0, 4) = (-projMat.at<float>(0, 3));
            // this->cameraMatrix.at<float>(0, 4) = (trans.at<float>(0));

            std::cout << "this->cameraMatrix: " << this->cameraMatrix << std::endl;

            file.close();
        }
    }

    ~DataSourceKittyStereo()
    {
        if (timeFileStream.is_open())
        {
            timeFileStream.close();
        }
    }

    T GetNext() const override;

    cv::Mat GetParameters() const override;
    cv::Mat GetDistortion() const override;

private:
    const std::string folderLeft;
    const std::string folderRight;
    const std::string timestampFolder;
    mutable std::ifstream timeFileStream;

    cv::Mat_<float> cameraMatrix;

    mutable unsigned id;
};

}

namespace data_source
{

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
T DataSourceKittyStereo<T>::GetNext() const
{
    const auto im1 = cv::imread(folderLeft  + format(id) + ".png");
    const auto im2 = cv::imread(folderRight + format(id) + ".png");
    double timestamp = -1.0;

    if (timeFileStream.is_open())
    {
        //std::string timestampStr;

        timeFileStream >> timestamp;
    }

    ++id;

    return {timestamp, im1.clone(), im2.clone()};
}

template<typename T>
cv::Mat DataSourceKittyStereo<T>::GetParameters() const
{
    return cameraMatrix;
}

template<typename T>
cv::Mat DataSourceKittyStereo<T>::GetDistortion() const
{
    // cv::Mat dist(1, 5);
    // dist.zero();
    return cv::Mat::zeros(1, 5, CV_32F);
}



}