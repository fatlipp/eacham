#pragma once

#include "odometry/frame/Frame.h"
#include "MotionEstimatorBase.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

namespace eacham
{
class Cal3_S2;

class MotionEstimatorOpt : public MotionEstimatorBase
{

public:
    MotionEstimatorOpt(const FeatureExtractorType &featureExtractor, const cv::Mat &cameraMat, const cv::Mat &distCoeffs);

    std::tuple<Eigen::Matrix4f, unsigned> Estimate(const Frame& frame1, Frame& frame2) override;

private:
    boost::shared_ptr<gtsam::Cal3_S2> K;
};
    
}