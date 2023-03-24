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
    FeatureExtractor(const FeatureExtractorType &typeInp)
        : type(typeInp)
    {
        switch (type)
        {
        case FeatureExtractorType::ORB:
            this->detector = cv::ORB::create(1000, 1.6f, 6, 19);
            break;
        case FeatureExtractorType::SIFT:
            this->detector = cv::SIFT::create(1000);
            break;
        case FeatureExtractorType::SURF:
            this->detector = cv::xfeatures2d::SURF::create(1000);
            break;
        }
    }

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