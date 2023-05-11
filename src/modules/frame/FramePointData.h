#pragma once

#include "types/DataTypes.h"

#include <opencv2/core.hpp>

namespace eacham
{

struct FramePointData
{
    unsigned id;
    cv::Point2f keypoint;
    cv::Point3f position3d;
    cv::Mat descriptor;
};

}