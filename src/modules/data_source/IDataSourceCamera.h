#pragma once

#include <string>
#include <tuple>
#include <opencv2/opencv.hpp>

#include "data_source/IDataSource.h"


namespace eacham
{

template<typename T>
class IDataSourceCamera : public IDataSource<T>
{

public:
    virtual bool isStereo() const = 0;
    virtual bool isRgbd() const = 0;
    virtual cv::Mat GetParameters() const = 0;
    virtual cv::Mat GetDistortion() const = 0;
};

}