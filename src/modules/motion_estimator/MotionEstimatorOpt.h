#pragma once

#include "MotionEstimatorBase.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

namespace eacham
{

class MotionEstimatorOpt : public MotionEstimatorBase
{

public:
    MotionEstimatorOpt(const cv::Mat &cameraMat, const cv::Mat &distCoeffs);

    EstimationResult Estimate(const IFrameLight& frame1, const IFrame& frame2,
        const std::vector<std::pair<unsigned, unsigned>>& matches) override;

private:
    boost::shared_ptr<gtsam::Cal3_S2> cameraMatGtsam;
};
    
}