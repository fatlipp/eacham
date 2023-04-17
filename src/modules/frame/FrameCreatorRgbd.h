#pragma once

#include "frame/IFrameCreator.h"

namespace eacham
{

class FrameCreatorRgbd : public IFrameCreator
{
public:
    FrameCreatorRgbd(const cv::Mat &cameraData, extractor_t&& extractor) 
        : IFrameCreator(cameraData, std::move(extractor))
    {
    }
protected:
    IFrame Create(const stereodata_t& data) override;
};

} // namespace eacham
