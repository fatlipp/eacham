#pragma once

#include "core/data_source/IDataSource.h"
#include "core/data_source/DataSourceTypes.h"
#include "core/types/DataTypes.h"

#include <string>
#include <tuple>
#include <future>
#include <thread>
#include <atomic>
#include <opencv2/opencv.hpp>


namespace eacham
{

template<typename... Ts>
class ICamera : public IDataSource<Ts...>
{
public:
    ICamera()
        : type(CameraType::MONO)
        {
        }

    virtual bool Initialize() { return true; }

public:
    typename IDataSource<Ts...>::ReturnType Get() const override
    {
        std::lock_guard<std::mutex> lock(this->dataMutex);
        return data;
    }

    virtual void Read() = 0;
    virtual bool HasNext() const = 0;

public:
    cv::Mat GetParameters() const
    {
        return this->cameraMatrix;
    }

    cv::Mat GetDistortion() const
    {
        return this->distMatrix;
    }

protected:
    const CameraType type;

    cv::Mat cameraMatrix;
    cv::Mat distMatrix;

    mutable std::mutex dataMutex;
    typename IDataSource<Ts...>::ReturnType data;
};

}