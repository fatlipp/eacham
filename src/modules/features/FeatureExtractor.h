#pragma once

#include "types/DataTypes.h"
#include "FeatureExtractorType.h"
#include "config/ConfigFeatureExtractor.h"

#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

namespace eacham
{

class FeatureExtractor
{
public:
    FeatureExtractor(const ConfigFeatureExtractor &config);

    std::tuple<std::vector<cv::KeyPoint>, cv::Mat> GetFeatures(const cv::Mat &image);

private:
    cv::Ptr<cv::FeatureDetector> detector;
};

}