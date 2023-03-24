#pragma once

#include "odometry/frame/Frame.h"

namespace eacham
{

class IMotionEstimator
{
public:
    ~IMotionEstimator() = default;

    virtual std::tuple<Eigen::Matrix4f, unsigned> Estimate(const Frame& frame1, Frame& frame2) = 0;
};

}