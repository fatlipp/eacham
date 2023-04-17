#pragma once

#include "frame/IFrame.h"

namespace eacham
{

class IMotionEstimator
{
public:
    ~IMotionEstimator() = default;

    virtual std::tuple<Eigen::Matrix4f, unsigned> Estimate(const IFrame& frame1, IFrame& frame2) = 0;
};

}