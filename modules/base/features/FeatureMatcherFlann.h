#pragma once

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/calib3d.hpp>

#include <vector>

namespace eacham
{

class FeatureMatcherFlann
{
public:
    using MatchType = std::vector<std::pair<unsigned, unsigned>>;
public:
    FeatureMatcherFlann();

public:
    MatchType Match(const cv::Mat& descriptor1, const cv::Mat& descriptor2);

private:
    cv::Ptr<cv::DescriptorMatcher> mather;
};

}