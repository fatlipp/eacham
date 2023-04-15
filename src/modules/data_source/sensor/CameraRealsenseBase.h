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

        std::tie(this->cameraMatrix, this->distMatrix) = this->GenerateIntrinsics(config);

        this->isInitialized = true;

        std::cout << "Realsense Stereo cameraMatrix:\n" << this->cameraMatrix << std::endl;
        std::cout << "Realsense Stereo distMatrix:\n" << this->distMatrix << std::endl;
    }

protected:
    virtual rs2::config GetRealsenseConfig(const ConfigCamera& config) const = 0;
    virtual std::tuple<cv::Mat, cv::Mat> GenerateIntrinsics(const ConfigCamera& config) const = 0;

    void Process() override
    {
        rs2::frameset frames = this->pipeline.wait_for_frames();
        rs2::frame frameLeft;
        rs2::frame frameRight;
        switch (this->type)
        {
            case CameraType::MONO:
                frameLeft = frames.first(RS2_STREAM_INFRARED);
                break;
            case CameraType::RGBD:
                frameLeft = frames.first(RS2_STREAM_INFRARED);
                frameRight = frames.get_depth_frame();
                // frameRight = this->thresholdFilter.process(frameRight);
                break;
            case CameraType::STEREO:
                frameLeft = frames.get_infrared_frame(1);
                frameRight = frames.get_infrared_frame(2);
                break;
        }

        SaveData(frameLeft, frameRight);
    }

    void SaveData(const rs2::frame &left, const rs2::frame &right)
    {
        std::lock_guard<std::mutex> lock(this->dataMutex);
        switch (this->type)
        {
            case CameraType::MONO:
                this->imageLeft = cv::Mat(cv::Size(this->width, this->height), CV_8UC1, (void*)left.get_data(), cv::Mat::AUTO_STEP);
                break;
            case CameraType::RGBD:
                this->imageLeft = cv::Mat(cv::Size(this->width, this->height), CV_8UC1, (void*)left.get_data(), cv::Mat::AUTO_STEP);
                this->imageRight = cv::Mat(cv::Size(this->width, this->height), CV_16U, (void*)right.get_data(), cv::Mat::AUTO_STEP);
                break;
            case CameraType::STEREO:
                this->imageLeft = cv::Mat(cv::Size(this->width, this->height), CV_8UC1, (void*)left.get_data(), cv::Mat::AUTO_STEP);
                this->imageRight = cv::Mat(cv::Size(this->width, this->height), CV_8UC1, (void*)right.get_data(), cv::Mat::AUTO_STEP);
                break;
        }

        this->dataGot = true;
    }

protected:
    bool isInitialized;
    rs2::pipeline pipeline;
    rs2::pipeline_profile profile;

    unsigned width;
    unsigned height;

};

}