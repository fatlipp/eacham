#pragma once

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/calib3d.hpp>

#include <map>

namespace eacham
{

class FeatureMatcherFlann
{
public:
    using MatchType = std::unordered_map<unsigned, unsigned>;
public:
    FeatureMatcherFlann(const float inliersRatio);

public:
    MatchType Match(const cv::Mat& descriptor1, const cv::Mat& descriptor2);

private:
    float inliersRatio;
    cv::Ptr<cv::DescriptorMatcher> mather;
};

}