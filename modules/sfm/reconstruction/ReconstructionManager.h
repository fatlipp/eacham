#pragma once

#include "base/tools/Tools3d.h"

#include "sfm/data/Map.h"
#include "sfm/data/Types.h"
#include "sfm/data/Graph.h"

#include <opencv4/opencv2/opencv.hpp>

namespace eacham
{

class ReconstructionManager
{

public:
    ReconstructionManager(std::shared_ptr<graph_t> graph, std::shared_ptr<Map> map,
        const float maxReprError, const float minTriAngle, const int minPnpInliers)
        : graph{graph}
        , map{map}
        , maxReprError{maxReprError}
        , minTriAngle{minTriAngle}
        , minPnpInliers{minPnpInliers}
        {}

public:
    MatchTwoView RecoverPoseTwoView(const unsigned id1, const unsigned id2, 
        const cv::Mat& K) const;

    bool RecoverPosePnP(const unsigned id1, const unsigned id2, const cv::Mat& K);

private:
    const std::shared_ptr<graph_t> graph;
    const std::shared_ptr<Map> map;
    const float maxReprError;
    const float minTriAngle;
    const int minPnpInliers;
};

} // namespace eacham