#pragma once

#include "odometry/frame/IFrameCreator.h"

namespace eacham
{

class FrameCreatorStereo : public IFrameCreator
{
public:
    FrameCreatorStereo(const FeatureExtractorType& featureExtractor) : IFrameCreator(featureExtractor)
    {}
    
protected:
    Frame Create(const stereodata_t& data, const cv::Mat &camera) override;
};

} // namespace eacham
