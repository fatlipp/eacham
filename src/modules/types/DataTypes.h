#pragma once

#include <tuple>
#include <memory>

#include <opencv2/opencv.hpp>

using stereodata_t = std::tuple<double, cv::Mat, cv::Mat>;

using matcher_t = cv::Ptr<cv::DescriptorMatcher>;