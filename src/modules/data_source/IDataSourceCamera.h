#pragma once

#include <string>
#include <tuple>
#include <opencv2/opencv.hpp>

#include "data_source/IDataSource.h"
#include "data_source/DataSourceTypes.h"
#include "config/Config.h"


namespace eacham
{

template<typename T>
class IDataSourceCamera : public IDataSource<T>
{
public:
    IDataSourceCamera(const CameraType& type)
        : type(type)
        {
        }

public:
    virtual void Initialize(const ConfigCamera& config) = 0;

public:
    virtual cv::Mat GetParameters() const = 0;
    virtual cv::Mat GetDistortion() const = 0;

public:
    bool isStereo() const
    {
        return this->type == CameraType::STEREO;
    }

    bool isRgbd() const
    {
        return this->type == CameraType::RGBD;
    }

protected:
    const CameraType type;
};

}