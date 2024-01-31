#pragma once

#include "base/features/IFeatureExtractor.h"

#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"

namespace eacham
{
struct FeatureExtractorSiftTypes
{
    using feature_t = std::vector<cv::Point2f>;
    using descriptor_t = cv::Mat;
};

class FeatureExtractorSift : 
    public IFeatureExtractor<FeatureExtractorSiftTypes::feature_t, FeatureExtractorSiftTypes::descriptor_t>
{
public:
    FeatureExtractorSift(const int maxFeatures); 

    typename FeatureExtractorSift::ReturnType Extract(const cv::Mat &image) override;

private:
    cv::Ptr<cv::FeatureDetector> detector;
};

}