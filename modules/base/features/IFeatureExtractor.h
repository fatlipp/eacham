#pragma once

#include <opencv2/opencv.hpp>

namespace eacham
{

template<typename TK, typename TD>
class IFeatureExtractor
{
public:
    using FeatureType = TK;
    using DescriptorType = TD;
    using ReturnType = std::pair<FeatureType, DescriptorType>;

public:
    virtual ~IFeatureExtractor() = default;

    virtual ReturnType Extract(const cv::Mat &image) = 0;
};

}