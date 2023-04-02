#pragma once

#include "types/DataTypes.h"
#include "FeatureExtractorType.h"
#include "FeatureMatcherFactory.h"

#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

namespace eacham
{

class FeatureExtractor
{
public:
    FeatureExtractor(const FeatureExtractorType &typeInp);

    std::tuple<std::vector<cv::KeyPoint>, cv::Mat> GetFeatures(const cv::Mat &image);

    matcher_t CreateMatcher()
    {
        return CreateFeatureMatcher(type);
    }


private:
    const FeatureExtractorType type;

    cv::Ptr<cv::FeatureDetector> detector;
};

}