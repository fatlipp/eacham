#pragma once

#include <tuple>
#include <memory>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>

namespace eacham
{
class FeatureExtractor;

template<typename T>
class IDataSourceCamera;

using stereodata_t = std::tuple<double, cv::Mat, cv::Mat>;
using lidardata_t = std::vector<Eigen::Vector3f>;

using matcher_t = cv::Ptr<cv::DescriptorMatcher>;
using extractor_t = std::unique_ptr<FeatureExtractor>;

using camera_t = IDataSourceCamera<stereodata_t>;
using camera_ptr_t = std::unique_ptr<camera_t>;

}
