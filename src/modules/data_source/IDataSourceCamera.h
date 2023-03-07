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
    IDataSourceCamera() = default;

    virtual ~IDataSourceCamera() = default;

    virtual T Get() const = 0;

    virtual cv::Mat GetParameters() const = 0;
    virtual cv::Mat GetDistortion() const = 0;
};

}