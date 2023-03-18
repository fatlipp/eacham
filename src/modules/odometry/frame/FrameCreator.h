#pragma once

#include "Frame.h"
#include "types/DataTypes.h"
#include "odometry/features/FeatureExtractor.h"

namespace eacham
{

class FrameCreator
{
public:
    FrameCreator()
    {
        this->mather = FeatureExtractor::GetMatcher();
    }

    Frame Create(const stereodata_t& data, const cv::Mat &camera);

private:
    matcher_t mather;

    FeatureExtractor extractor;

};

} // namespace eacham
