#pragma once

#include "sfm/config/SfmConfig.h"
#include "sfm/data/Map.h"
#include "sfm/data/Types.h"
#include "sfm/data/Graph.h"

#include <unordered_set>

namespace eacham
{

void RefineBA(
    const int currentFrameId,
    std::shared_ptr<graph_t> graph, 
    std::shared_ptr<Map> map, cv::Mat& K, 
    const OptimizerConfig& config);

}