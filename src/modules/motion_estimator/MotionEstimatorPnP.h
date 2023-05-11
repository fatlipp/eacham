#pragma once

#include "MotionEstimatorBase.h"

namespace eacham
{
class MotionEstimatorPnP : public MotionEstimatorBase
{
public:
    MotionEstimatorPnP(const cv::Mat &cameraMatInp, const cv::Mat &distCoeffsInp)
        : MotionEstimatorBase(cameraMatInp, distCoeffsInp)
        {
        }

public:
    EstimationResult Estimate(const IFrameLight& frame1, const IFrame& frame2,
        const std::vector<std::pair<unsigned, unsigned>>& matches) override;

};
    
}