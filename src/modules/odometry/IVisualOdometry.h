#pragma once

#include "odometry/IOdometry.h"
#include "frame/FrameMatcher.h"
#include "motion_estimator/IMotionEstimator.h"

#include <Eigen/Core>

namespace eacham
{

class IVisualOdometry : public IOdometry
{
public:
    IVisualOdometry(std::unique_ptr<FrameMatcher> &frameMatcher)
        : frameMatcher(std::move(frameMatcher))
    { }

public:
    void SetMotionEstimator(std::unique_ptr<IMotionEstimator> motionEstimatorInp)
    {
        this->motionEstimator = std::move(motionEstimatorInp);
    }

protected:
    std::unique_ptr<FrameMatcher> frameMatcher;
    std::unique_ptr<IMotionEstimator> motionEstimator;
};

} // namespace eacham
