#pragma once

#include "IFrame.h"
#include "types/DataTypes.h"
#include "features/FeatureExtractor.h"

namespace eacham
{

class IFrameCreator
{
public:

    IFrameCreator(const cv::Mat &cameraData, extractor_t extractor)
        : cameraData(cameraData)
        , extractor(std::move(extractor))
    {
    }

    virtual IFrame Create(const stereodata_t& data) = 0;

protected:
    unsigned GetId() const
    {
        static unsigned ID = 1;
        
        return ID++;
    }
    
protected:
    cv::Mat cameraData;
    extractor_t extractor;
};

} // namespace eacham
