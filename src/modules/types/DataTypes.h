#pragma once

#include <tuple>
#include <memory>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>

namespace eacham
{
    struct CameraDataOne
    {
        double timestamp;
        cv::Mat left;
    };

    struct CameraDataTwo
    {
        double timestamp;
        cv::Mat left;
        cv::Mat right;
    };

    struct CameraDataTwoDataset
    {
        double timestamp;
        cv::Mat left;
        cv::Mat right;
        Eigen::Matrix4f gtPos;
    };
}

namespace eacham
{
class FeatureExtractor;

template<typename T>
class IDataSourceCamera;

using monodata_t = std::tuple<double, cv::Mat>;
using stereodata_t = std::tuple<double, cv::Mat, cv::Mat>;
using lidardata_t = std::vector<Eigen::Vector3f>;

using matcher_t = cv::Ptr<cv::DescriptorMatcher>;
using extractor_t = std::unique_ptr<FeatureExtractor>;

}
