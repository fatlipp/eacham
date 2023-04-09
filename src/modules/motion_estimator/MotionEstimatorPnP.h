#pragma once

#include "frame/Frame.h"
#include "MotionEstimatorBase.h"

namespace eacham
{
class MotionEstimatorPnP : public MotionEstimatorBase
{

public:
    MotionEstimatorPnP(const cv::Mat &cameraMat, const cv::Mat &distCoeffs);

    std::tuple<Eigen::Matrix4f, unsigned> Estimate(Frame& frame1, Frame& frame2) override;

};
    
}