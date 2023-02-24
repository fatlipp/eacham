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

    void Estimate(const Frame& frame1, const Frame& frame2);
};

MotionEstimator::MotionEstimator()
{
}

MotionEstimator::~MotionEstimator()
{
}

void MotionEstimator::Estimate(const Frame& frame1, const Frame& frame2)
{
    
}
    
}