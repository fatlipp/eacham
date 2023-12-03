#pragma once

#include <opencv2/opencv.hpp>

namespace eacham
{

template<typename TE, typename TM>
class FeaturePipelineLightGlue
{
public:
    TE::ReturnType Extract(const cv::Mat &image)
    {
        return extractor.Extract(image);
    }

    TM::MatchType Match(const std::pair<typename TE::FeatureType, typename TE::DescriptorType>& descriptor1, 
                        const std::pair<typename TE::FeatureType, typename TE::DescriptorType>& descriptor2)
    {
        return matcher.Match(descriptor1, descriptor2);
    }

protected:
    TE extractor;
    TM matcher;
};

}