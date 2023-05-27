#pragma once

#include <vector>
#include <list>

// #include <pcl/common/eigen.h>
// #include <pcl/common/common.h>

#include "odometry/IOdometry.h"
#include "odometry/IVisualOdometry.h"
#include "odometry/FrameToFrameOdometry.h"
#include "config/Config.h"
#include "tools/Tools3d.h"
#include "types/DataTypes.h"
#include "data_source/IDataSourceCamera.h"
#include "frame/IFrameCreator.h"
#include "frame/FrameCreatorRgbd.h"
#include "frame/FrameCreatorStereo.h"
#include "frame/FrameMatcher.h"
#include "motion_estimator/MotionEstimatorPnP.h"
#include "motion_estimator/MotionEstimatorOpt.h"
#include "motion_estimator/IMotionEstimator.h"
#include "motion_estimator/MotionEstimatorType.h"

namespace eacham
{

template<typename T>
class VisualOdometryDirector
{
using camera_t = IDataSourceCamera<T>;

public:
    std::unique_ptr<IVisualOdometry> Build(const camera_t* const camera, const Config& config)
    {   
        auto configOdometry = config.GetOdometry();
        auto featureMatcher = BuildFeatureMatcher(config.GetFeatureExtractor().GetType());
        auto frameMatcher = std::make_unique<FrameMatcher>(featureMatcher);

        std::unique_ptr<IVisualOdometry> odometry = std::make_unique<FrameToFrameOdometry>(frameMatcher);

        if (odometry != nullptr)
        {
            odometry->SetMotionEstimator(BuildMotionEstimator(camera, configOdometry.motionEstimatorType));
        }

        return odometry;
    }

    matcher_t BuildFeatureMatcher(const FeatureExtractorType &type)
    {
        switch (type)
        {
        case FeatureExtractorType::ORB:
            return cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
        
        default:
            return cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
            // return cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        }

        return nullptr;
    }

    std::unique_ptr<IMotionEstimator> BuildMotionEstimator(const camera_t* const camera,
        const MotionEstimatorType &type)
    {
        std::unique_ptr<MotionEstimatorBase> motionEstimator;
        switch (type)
        {
        case MotionEstimatorType::OPT:
            motionEstimator = std::make_unique<MotionEstimatorOpt>(camera->GetParameters(), camera->GetDistortion());
            break;

        default:
            motionEstimator = std::make_unique<MotionEstimatorPnP>(camera->GetParameters(), camera->GetDistortion());
        }

        return motionEstimator;
    }
};

} // namespace eacham