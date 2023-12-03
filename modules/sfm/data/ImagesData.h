#pragma once

#include "data/Types.h"
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <Eigen/Core>

#include <map>

namespace eacham
{

struct Frame
{
    unsigned id;
    cv::Mat image;
};

}