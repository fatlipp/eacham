#pragma once

#include "frame/IFrame.h"
#include "frame/IFrameLight.h"
#include "motion_estimator/EstimationResult.h"

namespace eacham
{

class IMotionEstimator
{
public:
    ~IMotionEstimator() = default;

    virtual EstimationResult Estimate(const IFrameLight& frame1, const IFrame& frame2,
        const std::vector<std::pair<unsigned, unsigned>>& matches) = 0;
};

}