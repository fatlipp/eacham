
#pragma once

#include <opencv2/core.hpp>

namespace eacham
{

struct MapPoint
{
    unsigned id = 0;
    unsigned observers = 0;
    cv::Point3f position;

    MapPoint(const unsigned id, const cv::Point3f &position)
        : id(id)
        , position(position)
        {}

    void AddObserver() noexcept
    {
        ++observers;
    }
};

}