#pragma once

#include <vector>
#include <list>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include "odometry/IVisualOdometry.h"
#include "odometry/FrameToMapOdometry.h"
#include "config/Config.h"
#include "tools/Tools3d.h"
#include "types/DataTypes.h"
#include "map/LocalMap.h"
#include "data_source/IDataSourceCamera.h"
#include "frame/IFrameCreator.h"
#include "frame/FrameCreatorRgbd.h"
#include "frame/FrameCreatorStereo.h"
#include "optimization/LocalFramesOptimizer.h"
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
    std::unique_ptr<IVisualOdometry<T>> Build(const camera_t* const camera, const ConfigOdometry& config)
    {
        auto featureMatcher = BuildFeatureMatcher(config.featureExtractorType);

        std::unique_ptr<IVisualOdometry<T>> odometry;

        // TODO: different visual odometry types
        switch (config.odometryType)
        {
            case OdometryType::FRAME_TO_FRAME:
                odometry = std::make_unique<FrameToMapOdometry<T>>();
                break;
            case OdometryType::FRAME_TO_MAP:
                odometry = std::make_unique<FrameToMapOdometry<T>>();
                break;
            case OdometryType::OPT:
                odometry = std::make_unique<FrameToMapOdometry<T>>();
                break;
        }

        odometry->SetFrameCreator(BuildFrameCreator(camera, featureMatcher, config.featureExtractorType));
        odometry->SetMotionEstimator(BuildMotionEstimator(camera, config.motionEstimatorType, featureMatcher));
        // odometry->SetLocalOptimizer(BuildLocalOptimizer(camera));

        return odometry;
    }

    extractor_t BuildFeatureExtractor(const FeatureExtractorType &type)
    {
        return std::make_unique<FeatureExtractor>(type);
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

    std::unique_ptr<IFrameCreator> BuildFrameCreator(const camera_t* const camera, const matcher_t& matcher, const FeatureExtractorType &type)
    {
        auto featureExtractor = BuildFeatureExtractor(type);

        if (camera->isStereo())
        {
            return BuildFrameCreatorStereo(camera->GetParameters(), std::move(featureExtractor), matcher);
        }

        return BuildFrameCreatorRgbd(camera->GetParameters(), std::move(featureExtractor));
    }

    std::unique_ptr<IFrameCreator> BuildFrameCreatorRgbd(const cv::Mat &cameraParameters, extractor_t&& extractor)
    {
        return std::make_unique<FrameCreatorRgbd>(cameraParameters, std::move(extractor));
    }

    std::unique_ptr<IFrameCreator> BuildFrameCreatorStereo(const cv::Mat &cameraParameters, extractor_t&& extractor, const matcher_t& matcher)
    {
        return std::make_unique<FrameCreatorStereo>(cameraParameters, std::move(extractor), matcher);
    }

    std::unique_ptr<IMotionEstimator> BuildMotionEstimator(const camera_t* const camera, const MotionEstimatorType &type, const matcher_t& matcher)
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

        motionEstimator->SetMatcher(matcher);

        return motionEstimator;
    }

    std::unique_ptr<LocalFramesOptimizer> BuildLocalOptimizer(const camera_t* const camera)
    {
        return std::make_unique<LocalFramesOptimizer>(camera->GetParameters(), camera->GetDistortion());
    }
};

} // namespace eacham