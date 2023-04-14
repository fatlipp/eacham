#pragma once

#include <string>
#include <tuple>
#include <opencv2/opencv.hpp>

#include "data_source/IDataSourceCamera.h"
#include "config/Config.h"

#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API

namespace eacham
{

template<typename T>
class CameraRealsenseBase : public IDataSourceCamera<T>
{
public:
    CameraRealsenseBase(const CameraType& type)
        : IDataSourceCamera<T>(type)
        , isInitialized(false)
    {
    }

public:
    void Initialize(const ConfigCamera& config) override
    {
        if (this->isInitialized)
        {
            return;
        }

        this->width = config.width;
        this->height = config.height;

        const auto realsenseConfig = GetRealsenseConfig(config);
        this->profile = pipeline.start(realsenseConfig);
        
        rs2::device selected_device = this->profile.get_device();
        auto depth_sensor = selected_device.first<rs2::depth_sensor>();

        if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
        }

        std::tie(this->cameraMatrix, this->distMatrix) = GenerateIntrinsics(config);

        this->isInitialized = true;

        std::cout << "Realsense Stereo cameraMatrix:\n" << this->cameraMatrix << std::endl;
        std::cout << "Realsense Stereo distMatrix:\n" << this->distMatrix << std::endl;
    }

public:
    cv::Mat GetParameters() const override
    {
        return this->cameraMatrix;
    }

    cv::Mat GetDistortion() const override
    {
        return this->distMatrix;
    }

protected:
    virtual rs2::config GetRealsenseConfig(const ConfigCamera& config) const = 0;
    virtual std::tuple<cv::Mat, cv::Mat> GenerateIntrinsics(const ConfigCamera& config) const = 0;

protected:
    bool isInitialized;
    rs2::pipeline pipeline;
    rs2::pipeline_profile profile;

    cv::Mat cameraMatrix;
    cv::Mat distMatrix;

    unsigned width;
    unsigned height;

};

}