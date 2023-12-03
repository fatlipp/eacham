#pragma once

#include <string>
#include <tuple>
#include <opencv2/opencv.hpp>

#include "data_source/sensor/CameraRealsenseBase.h"

namespace eacham
{

template<typename T>
class CameraRealsenseRgbd : public CameraRealsenseBase<T>
{
public:
    CameraRealsenseRgbd()
        : CameraRealsenseBase<T>(CameraType::RGBD)
        {
        }

public:
    void Initialize(const ConfigCamera& config) override
    {
        CameraRealsenseBase<T>::Initialize(config);
        
        this->thresholdFilter.set_option(RS2_OPTION_MIN_DISTANCE, 0.1f);
        this->thresholdFilter.set_option(RS2_OPTION_MAX_DISTANCE, 15.0f);

        // this->temporalFilter.set_option(RS2_OPTION_HOLES_FILL, 3);
        // this->temporalFilter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
        // this->temporalFilter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    }

protected:
    rs2::config GetRealsenseConfig(const ConfigCamera& config) const override
    {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_INFRARED, config.width, config.height, RS2_FORMAT_Y8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, config.width, config.height, RS2_FORMAT_Z16, 30);

        return cfg;
    }
    
    std::tuple<cv::Mat, cv::Mat> GenerateIntrinsics(const ConfigCamera& config) const override
    {
        rs2::stream_profile cameraStream = this->profile.get_stream(RS2_STREAM_INFRARED);
        rs2_intrinsics intrinsics = cameraStream.as<rs2::video_stream_profile>().get_intrinsics();

        cv::Mat cameraMatrix = cv::Mat(1, 5, CV_32F);
        cameraMatrix.at<float>(0, 0) = intrinsics.fx;
        cameraMatrix.at<float>(0, 1) = intrinsics.fy;
        cameraMatrix.at<float>(0, 2) = intrinsics.ppx;
        cameraMatrix.at<float>(0, 3) = intrinsics.ppy;
        cameraMatrix.at<float>(0, 4) = 1.0f / config.scale;

        cv::Mat distMatrix = cv::Mat(1, 5, CV_32F);
        distMatrix.at<float>(0, 0) = intrinsics.coeffs[0];
        distMatrix.at<float>(0, 1) = intrinsics.coeffs[1];
        distMatrix.at<float>(0, 2) = intrinsics.coeffs[2];
        distMatrix.at<float>(0, 3) = intrinsics.coeffs[3];
        distMatrix.at<float>(0, 4) = intrinsics.coeffs[4];

        return {cameraMatrix, distMatrix};
    }

private:
    rs2::threshold_filter thresholdFilter;
    rs2::temporal_filter temporalFilter;

};

}