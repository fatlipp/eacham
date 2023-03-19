#pragma once

#include "odometry/frame/Frame.h"
#include "MotionEstimatorBase.h"

namespace eacham
{
class MotionEstimatorPnP : public MotionEstimatorBase
{

public:
    MotionEstimatorPnP(const FeatureExtractorType &featureExtractor, const cv::Mat &cameraMat, const cv::Mat &distCoeffs);

    std::tuple<Eigen::Matrix4f, unsigned> Estimate(const Frame& frame1, Frame& frame2) override;

};
    
}