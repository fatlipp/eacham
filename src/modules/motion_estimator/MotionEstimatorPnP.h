#pragma once

#include "frame/IFrame.h"
#include "MotionEstimatorBase.h"

namespace eacham
{
class MotionEstimatorPnP : public MotionEstimatorBase
{

public:
    MotionEstimatorPnP(const cv::Mat &cameraMat, const cv::Mat &distCoeffs);

    std::tuple<Eigen::Matrix4f, unsigned> Estimate(const IFrame& frame1, IFrame& frame2) override;

};
    
}