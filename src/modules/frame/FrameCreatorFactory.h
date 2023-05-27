#pragma once

#include "frame/IFrameCreator.h"
#include "frame/FrameCreatorRgbd.h"
#include "frame/FrameCreatorStereo.h"

// #include <pcl/common/eigen.h>
// #include <pcl/common/common.h>

#include "odometry/IOdometry.h"
#include "odometry/IVisualOdometry.h"
#include "odometry/FrameToFrameOdometry.h"
#include "config/Config.h"
#include "tools/Tools3d.h"
#include "types/DataTypes.h"
#include "data_source/IDataSourceCamera.h"
#include "frame/FrameMatcher.h"
#include "motion_estimator/MotionEstimatorPnP.h"
#include "motion_estimator/MotionEstimatorOpt.h"
#include "motion_estimator/IMotionEstimator.h"
#include "motion_estimator/MotionEstimatorType.h"

namespace eacham
{

extractor_t BuildFeatureExtractor(const ConfigFeatureExtractor &config)
{
    return std::make_unique<FeatureExtractor>(config);
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

std::unique_ptr<IFrameCreator> BuildFrameCreatorStereo(const cv::Mat &cameraParameters,
    extractor_t&& extractor, const ConfigFeatureExtractor& config)
{
    auto featureMatcher = BuildFeatureMatcher(config.GetType());

    return std::make_unique<FrameCreatorStereo>(cameraParameters, std::move(extractor), featureMatcher);
}

std::unique_ptr<IFrameCreator> BuildFrameCreatorRgbd(const cv::Mat &cameraParameters, extractor_t&& extractor)
{
    return std::make_unique<FrameCreatorRgbd>(cameraParameters, std::move(extractor));
}

std::unique_ptr<IFrameCreator> BuildFrameCreator(const IDataSourceCamera<stereodata_t>* const camera, const Config& config)
{
    auto ext = BuildFeatureExtractor(config.GetFeatureExtractor());

    if (camera->isStereo())
    {
        return BuildFrameCreatorStereo(camera->GetParameters(), std::move(ext), config.GetFeatureExtractor());
    }

    return BuildFrameCreatorRgbd(camera->GetParameters(), std::move(ext));
}

}