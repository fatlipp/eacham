#pragma once

#include "odometry/frame/Frame.h"
#include "IMotionEstimator.h"

namespace eacham
{
class MotionEstimatorPnP : public IMotionEstimator
{

public:
    MotionEstimatorPnP();

    MotionEstimatorPnP(const cv::Mat &cameraMat, const cv::Mat &distCoeffs);

    std::tuple<Eigen::Matrix4f, unsigned> Estimate(const Frame& frame1, Frame& frame2) override;

private:
    cv::Mat cameraMat;
    cv::Mat cameraMatOneDim;
    cv::Mat distCoeffs;
};
    
}