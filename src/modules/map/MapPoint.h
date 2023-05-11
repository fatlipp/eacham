
#pragma once

#include <opencv2/core.hpp>

namespace eacham
{

struct MapPoint
{
    unsigned id = 0;
    cv::Point3f position;
    unsigned observers = 0;
};

}