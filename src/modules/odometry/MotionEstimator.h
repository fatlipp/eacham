#pragma once

#include "frame/Frame.h"

namespace odometry
{
class MotionEstimator
{

public:
    MotionEstimator();

    MotionEstimator(const cv::Mat &cameraMat, const cv::Mat &distCoeffs);
    ~MotionEstimator();

    std::tuple<Eigen::Matrix4f, unsigned> Estimate(const Frame& frame1, Frame& frame2);

private:
    cv::Mat cameraMat;
    cv::Mat cameraMatOneDim;
    cv::Mat distCoeffs;
};
    
}