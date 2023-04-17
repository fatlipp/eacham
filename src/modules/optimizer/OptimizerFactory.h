#pragma once

#include "config/Config.h"
#include "optimizer/IMapOptimizer.h"
#include "optimizer/MapOptimizerBA.h"

namespace eacham
{

class OptimizerFactory
{
public:
    static std::unique_ptr<IMapOptimizer> Build(const Config& config, const cv::Mat& cameraMat)
    {
        return std::make_unique<MapOptimizerBA>(cameraMat);
    }
};

} // namespace eacham