#pragma once

#include "base/tools/Tools3d.h"

#include "sfm/data/Map.h"
#include "sfm/data/Types.h"
#include "sfm/data/Graph.h"

#include <opencv4/opencv2/opencv.hpp>

namespace eacham
{

void RecoverPoseTwoView(const unsigned id1, const unsigned id2, 
    std::shared_ptr<graph_t> graph, 
    const cv::Mat& K,
    std::shared_ptr<Map> map, const float reprErrMax = 4.0f);

bool RecoverPosePnP(
    const unsigned id1, const unsigned id2,
    std::shared_ptr<graph_t> graph, 
    std::shared_ptr<Map> map, const cv::Mat& K);

} // namespace eacham