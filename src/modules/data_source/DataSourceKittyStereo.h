#pragma once

#include <opencv4/opencv2/opencv.hpp>
#include <algorithm>

#include "IDataSource.h"

namespace data_source
{

template<typename T>
class DataSourceKittyStereo : public IDataSource<T>
{
public:
    DataSourceKittyStereo(const std::string &sourcePath) 
        : folderLeft(sourcePath + "/dataset/sequences/00/image_0/")
        , folderRight(sourcePath + "/dataset/sequences/00/image_1/")
    {
    }

    T GetNext() const override;


private:
    const std::string folderLeft;
    const std::string folderRight;
    cv::Mat cameraMatrix;

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
    ++id;

    return {im1, im2};
}

}