#pragma once

#include <opencv2/opencv.hpp>

namespace eacham
{

template<typename TE, typename TM>
class FeaturePipelineCv
{
public:
    TE::ReturnType Extract(const cv::Mat &image)
    {
        return extractor.Extract(image);
    }

    TM::MatchType Match(const TE::DescriptorType& descriptor1, 
                        const TE::DescriptorType& descriptor2)
    {
        return matcher.Match(descriptor1, descriptor2);
    }

protected:
    TE extractor;
    TM matcher;
};

}