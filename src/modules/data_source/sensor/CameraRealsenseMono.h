#pragma once

#include <string>
#include <tuple>
#include <opencv2/opencv.hpp>

#include "data_source/sensor/CameraRealsenseBase.h"

namespace eacham
{

template<typename T>
class CameraRealsenseMono : public CameraRealsenseBase<T>
{
public:
    CameraRealsenseMono()
        : CameraRealsenseBase<T>(CameraType::MONO)
        {
        }

protected:
    rs2::config GetRealsenseConfig(const ConfigCamera& config) const override
    {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, config.width, config.height, RS2_FORMAT_Y8, 30);

        return cfg;
    }
    
    std::tuple<cv::Mat, cv::Mat> GenerateIntrinsics(const ConfigCamera& config) const override
    {
        const rs2::stream_profile cameraStream1 = this->profile.get_stream(RS2_STREAM_INFRARED, 1);
        const rs2_intrinsics intrinsics = cameraStream1.as<rs2::video_stream_profile>().get_intrinsics();
        
        cv::Mat cameraMatrix = cv::Mat(1, 5, CV_32F);
        cameraMatrix.at<float>(0, 0) = intrinsics.fx;
        cameraMatrix.at<float>(0, 1) = intrinsics.fy;
        cameraMatrix.at<float>(0, 2) = intrinsics.ppx;
        cameraMatrix.at<float>(0, 3) = intrinsics.ppy;
        cameraMatrix.at<float>(0, 4) = 1.0f;
        
        cv::Mat distMatrix = cv::Mat(1, 5, CV_32F);
        distMatrix.at<float>(0, 0) = intrinsics.coeffs[0];
        distMatrix.at<float>(0, 1) = intrinsics.coeffs[1];
        distMatrix.at<float>(0, 2) = intrinsics.coeffs[2];
        distMatrix.at<float>(0, 3) = intrinsics.coeffs[3];
        distMatrix.at<float>(0, 4) = intrinsics.coeffs[4];

        return {cameraMatrix, distMatrix};
    }
};

}