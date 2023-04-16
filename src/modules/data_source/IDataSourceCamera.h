#pragma once

#include <string>
#include <tuple>
#include <future>
#include <thread>
#include <opencv2/opencv.hpp>

#include "data_source/IDataSource.h"
#include "data_source/DataSourceTypes.h"
#include "config/Config.h"
#include "types/DataTypes.h"


namespace eacham
{

template<typename T>
class IDataSourceCamera : public IDataSource<T>
{
public:
    IDataSourceCamera(const CameraType& type)
        : type(type)
        , timestamp(0)
        , isRunning(false)
        , dataUpdated(false)
        {
        }

public:
    virtual void Initialize(const ConfigCamera& config) = 0;

    void Start()
    {
        this->isRunning = true;
        this->cameraThread = std::async(std::launch::async, &IDataSourceCamera<T>::Loop, this);
    }

    void Stop()
    {
        this->isRunning = false;
    }

public:
    T Get() const override
    {
        while (!this->dataUpdated)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }

        std::lock_guard<std::mutex> lock(this->dataMutex);
        this->dataUpdated = false;

        return {this->timestamp, this->imageLeft.clone(), this->imageRight.clone()};
    }

public:
    cv::Mat GetParameters() const
    {
        return this->cameraMatrix;
    }

    cv::Mat GetDistortion() const
    {
        return this->distMatrix;
    }

    bool isMono() const
    {
        return this->type == CameraType::MONO;
    }

    bool isStereo() const
    {
        return this->type == CameraType::STEREO;
    }

    bool isRgbd() const
    {
        return this->type == CameraType::RGBD;
    }

protected:
    void Loop()
    {
        while (this->isRunning)
        {
            Process();
        }
    }

    virtual void Process() = 0;

protected:
    const CameraType type;
    cv::Mat cameraMatrix;
    cv::Mat distMatrix;

    std::atomic<bool> isRunning;
    std::future<void> cameraThread;

    double timestamp;
    cv::Mat imageLeft;
    cv::Mat imageRight;
    mutable std::mutex dataMutex;
    mutable std::atomic<bool> dataUpdated;
};

}