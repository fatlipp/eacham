#pragma once

#include <tuple>
#include <memory>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>

using stereodata_t = std::tuple<double, cv::Mat, cv::Mat>;
using lidardata_t = std::vector<Eigen::Vector3f>;

using matcher_t = cv::Ptr<cv::DescriptorMatcher>;