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
class CameraRealsenseRgbd : public IDataSourceCamera<T>
{
public:
    CameraRealsenseRgbd()
        : isInitialized(false)
    {
    }

public:
    void Initialize(const ConfigCamera& config) override
    {
        if (!this->isInitialized)
        {
            rs2::config cfg;
            cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
            cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

            this->profile = pipeline.start(cfg);

            rs2::stream_profile cam_stream = this->profile.get_stream(RS2_STREAM_INFRARED);
            rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
            // std::cout << " height = " << intrinsics_cam.height << std::endl;
            // std::cout << " width = " << intrinsics_cam.width << std::endl;
            // std::cout << " Model = " << intrinsics_cam.model << std::endl;

            this->cameraMatrix = cv::Mat(1, 5, CV_32F);
            this->cameraMatrix.at<float>(0, 0) = intrinsics_cam.fx;
            this->cameraMatrix.at<float>(0, 1) = intrinsics_cam.fy;
            this->cameraMatrix.at<float>(0, 2) = intrinsics_cam.ppx;
            this->cameraMatrix.at<float>(0, 3) = intrinsics_cam.ppy;
            this->cameraMatrix.at<float>(0, 4) = 1.0f / config.scale;

            this->distMatrix = cv::Mat(1, 5, CV_32F);
            this->distMatrix.at<float>(0, 0) = intrinsics_cam.coeffs[0];
            this->distMatrix.at<float>(0, 1) = intrinsics_cam.coeffs[1];
            this->distMatrix.at<float>(0, 2) = intrinsics_cam.coeffs[2];
            this->distMatrix.at<float>(0, 3) = intrinsics_cam.coeffs[3];
            this->distMatrix.at<float>(0, 4) = intrinsics_cam.coeffs[4];
            
            this->isInitialized = true;


            rs2::device selected_device = this->profile.get_device();
            auto depth_sensor = selected_device.first<rs2::depth_sensor>();

            if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
            {
                depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
            }

        }
    }

    T Get() const override
    {
        rs2::frameset frames = pipeline.wait_for_frames();
        rs2::frame ir_frame = frames.first(RS2_STREAM_INFRARED);
        rs2::frame depth_frame = frames.get_depth_frame();

        cv::Mat imageLeft(cv::Size(640, 480), CV_8UC1, (void*)ir_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat imageDepth(cv::Size(640, 480), CV_16U, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        return {0, imageLeft, imageDepth};
    }
public:
    bool isStereo() const override
    {
        return false;
    }

    bool isRgbd() const override
    {
        return true;
    }

    cv::Mat GetParameters() const override
    {
        return cameraMatrix;
    }

    cv::Mat GetDistortion() const override
    {
        return cv::Mat::zeros(1, 5, CV_32F);
    }

private:
    bool isInitialized;
    rs2::pipeline pipeline;
    rs2::pipeline_profile profile;

    cv::Mat cameraMatrix;
    cv::Mat distMatrix;

};

}