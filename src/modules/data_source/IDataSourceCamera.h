#pragma once

#include <string>
#include <tuple>
#include <opencv2/opencv.hpp>

#include "data_source/IDataSource.h"
#include "config/Config.h"


namespace eacham
{

template<typename T>
class IDataSourceCamera : public IDataSource<T>
{

public:
    virtual void Initialize(const ConfigCamera& config) = 0;

public:
    virtual bool isStereo() const = 0;
    virtual bool isRgbd() const = 0;
    virtual cv::Mat GetParameters() const = 0;
    virtual cv::Mat GetDistortion() const = 0;
};

}