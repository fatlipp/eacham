#pragma once

#include "Frame.h"
#include "types/DataTypes.h"
#include "odometry/features/FeatureExtractor.h"

namespace odometry
{

class FrameCreator
{
public:
    FrameCreator()
    {
        this->mather = std::make_unique<cv::BFMatcher>(cv::NORM_HAMMING, false);
    }

    Frame Create(const stereodata_t& data, const cv::Mat &camera);

private:
    std::unique_ptr<cv::BFMatcher> mather;

    FeatureExtractor extractor;

};

} // namespace odometry
