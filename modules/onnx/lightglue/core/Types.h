#pragma once

#include <vector>
#include <array>

#include <opencv2/opencv.hpp>

namespace eacham
{

using point_t = cv::Point2f;
using keypoints_t = std::vector<point_t>;
using descriptor_t = std::vector<std::array<float, 256>>;
using matcher_t = std::pair<keypoints_t, descriptor_t>;

}