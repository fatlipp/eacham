#pragma once

#include "frame/IFrameCreator.h"

namespace eacham
{

class FrameCreatorStereo : public IFrameCreator
{
public:
    FrameCreatorStereo(const cv::Mat &cameraData, extractor_t&& extractor, const matcher_t& matcher) 
        : IFrameCreator(cameraData, std::move(extractor))
        , matcher(matcher)
    {}
    
protected:
    IFrame Create(const stereodata_t& data) override;

private:
    matcher_t matcher;
};

} // namespace eacham
