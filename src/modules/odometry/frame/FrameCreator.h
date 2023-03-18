#pragma once

#include "Frame.h"
#include "types/DataTypes.h"
#include "odometry/features/FeatureExtractor.h"

namespace eacham
{

class FrameCreator
{
public:
    FrameCreator(const FeatureExtractorType& featureExtractor)
    {
        this->extractor = std::make_unique<FeatureExtractor>(featureExtractor);
        this->mather = this->extractor->CreateMatcher();
    }

    Frame Create(const stereodata_t& data, const cv::Mat &camera);

private:
    std::unique_ptr<FeatureExtractor> extractor;
    matcher_t mather;
};

} // namespace eacham
