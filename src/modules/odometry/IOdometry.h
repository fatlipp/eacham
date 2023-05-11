#pragma once

#include "frame/IFrame.h"
#include "frame/IFrameLight.h"
#include "motion_estimator/EstimationResult.h"

#include <eigen3/Eigen/Core>

namespace eacham
{

class IOdometry
{
public:
    virtual ~IOdometry() = default;

public:
    virtual EstimationResult Process(const IFrame &frame) = 0;

protected:
    IFrameLight lastFrame;
};

} // namespace eacham
