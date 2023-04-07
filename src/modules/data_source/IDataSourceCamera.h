#pragma once

#include <string>
#include <tuple>
#include <opencv2/opencv.hpp>


namespace eacham
{

template<typename T>
class IDataSourceCamera
{
public:
    IDataSourceCamera() = default;

    virtual ~IDataSourceCamera() = default;

    virtual T Get() const = 0;

    virtual bool isStereo() const = 0;
    virtual bool isRgbd() const = 0;
    virtual cv::Mat GetParameters() const = 0;
    virtual cv::Mat GetDistortion() const = 0;
};

}