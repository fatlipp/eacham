#pragma once

#include "sfm/data/Map.h"
#include "sfm/data/Types.h"
#include "sfm/data/Graph.h"

#include <unordered_set>

namespace eacham
{

void EstimateUsingBA(std::shared_ptr<graph_t> graph, 
    std::shared_ptr<Map> map, cv::Mat& K, const unsigned maxIters = 100);

}