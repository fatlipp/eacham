#pragma once

#include <string>
#include <tuple>
#include <opencv4/opencv2/opencv.hpp>


namespace data_source
{

template<typename T>
class IDataSourceCamera
{
public:
    using ReturnType = T;

public:
    IDataSourceCamera() = default;

    virtual ~IDataSourceCamera() = default;

    virtual T GetNext() const = 0;

    virtual cv::Mat GetParameters() const = 0;
    virtual cv::Mat GetDistortion() const = 0;

private:
    const std::string sourcePath;
};

}