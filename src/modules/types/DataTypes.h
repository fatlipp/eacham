#pragma once

#include <tuple>
#include <memory>

#include <opencv4/opencv2/opencv.hpp>

using stereodata_t = std::tuple<double, cv::Mat, cv::Mat>;