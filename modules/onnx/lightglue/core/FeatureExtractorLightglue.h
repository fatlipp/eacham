#pragma once

#include "core/features/IFeatureExtractor.h"
#include "./Types.h"

#include <opencv2/opencv.hpp>

namespace eacham
{

class FeatureExtractorLightglue : public IFeatureExtractor<keypoints_t, descriptor_t>
{

public:
    FeatureExtractorLightglue::ReturnType Extract(const cv::Mat &image) override;
};

}