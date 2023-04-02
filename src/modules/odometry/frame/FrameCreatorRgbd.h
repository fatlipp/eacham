#pragma once

#include "odometry/frame/IFrameCreator.h"

namespace eacham
{

class FrameCreatorRgbd : public IFrameCreator
{
public:
    FrameCreatorRgbd(const FeatureExtractorType& featureExtractor) : IFrameCreator(featureExtractor)
    {
    }
protected:
    Frame Create(const stereodata_t& data, const cv::Mat &camera) override;
};

} // namespace eacham
