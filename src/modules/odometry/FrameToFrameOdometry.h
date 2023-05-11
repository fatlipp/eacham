#pragma once

#include "motion_estimator/EstimationResult.h"
#include "odometry/IVisualOdometry.h"
#include "frame/FrameMatcher.h"
#include "types/DataTypes.h"

#include <tuple>

namespace eacham
{

class FrameToFrameOdometry : public IVisualOdometry
{
public:
    FrameToFrameOdometry(std::unique_ptr<FrameMatcher> &frameMatcher)
        : IVisualOdometry(frameMatcher)
        {
        }

public:
    EstimationResult Process(const IFrame &frame) override;
};

} // namespace eacham

namespace eacham
{

EstimationResult FrameToFrameOdometry::Process(const IFrame &frame)
{
    if (!frame.isValid())
    {
        std::cout << "\n++++++++++\nMotion estimation error: Invalid frame\n++++++++++\n";

        return { .frameIdPrev = 0, .frameIdCurrent = 0 };
    }

    if (!this->lastFrame.isValid())
    {
        this->lastFrame = { frame.GetId(), frame.GetPointsDataCopy() };

        return EstimationResult { .frameIdPrev = 0, .frameIdCurrent = frame.GetId(), .odometry = Eigen::Matrix4f::Identity() };
    }

    const auto matches = this->frameMatcher->FindMatches(this->lastFrame, frame);
    std::cout << "FrameToFrameOdometry() matches: " << matches.size() << std::endl;

    Eigen::Affine3f motion = Eigen::Affine3f::Identity();

    const int MIN_INLIERS = 5;

    if (matches.size() < MIN_INLIERS)
    {
        std::cout << "FrameToFrameOdometry() No enough inliers (" << matches.size() << ")" << std::endl;
        return EstimationResult { .frameIdPrev = 0, .frameIdCurrent = 0, .odometry = Eigen::Matrix4f::Identity() };
    }

    const auto res = this->motionEstimator->Estimate(this->lastFrame, frame, matches);

    if (res.isValid())
    {
        this->lastFrame = { frame.GetId(), frame.GetPointsDataCopy() };
    }
    std::cout << "TOTAL matches: " << res.matches.size() << std::endl;

    return res;
}

}