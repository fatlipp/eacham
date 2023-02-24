#include "frame/Frame.h"

#pragma once

namespace odometry
{
class MotionEstimator
{
private:
    /* data */
public:
    MotionEstimator();
    ~MotionEstimator();

    Eigen::Matrix4f Estimate(const Frame& frame1, const Frame& frame2);
};
    
}