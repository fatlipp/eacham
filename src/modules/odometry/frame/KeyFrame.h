#pragma once

#include "odometry/frame/Frame.h"
#include "types/DataTypes.h"

namespace odometry
{

class KeyFrame : public Frame
{
public:
    KeyFrame(const std::tuple<std::vector<cv::KeyPoint>, cv::Mat> &features)
        : Frame(-1.0, {}, features)
    {
    }
};

} // namespace odometry
