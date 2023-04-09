#pragma once

#include "types/DataTypes.h"
#include "FeatureExtractorType.h"

#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

namespace eacham
{

static matcher_t CreateFeatureMatcher(const FeatureExtractorType &type)
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

}