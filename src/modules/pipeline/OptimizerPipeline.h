#pragma once

#include "pipeline/IPipeline.h"
#include "data_source/IDataSource.h"
#include "frame/IFrameCreator.h"
#include "frame/FramePointData.h"
#include "odometry/IOdometry.h"
#include "map/Map.h"
#include "map/MapFrame.h"
#include "map/MapPoint.h"
#include "optimizer/IMapOptimizer.h"
#include "optimizer/MapOptimizerBA.h"
#include "motion_estimator/IMotionEstimator.h"
#include "motion_estimator/EstimationResult.h"
#include "performance/BlockTimer.h"
#include "data_source/dataset/IDataset.h"

#include <future>
#include <atomic>
#include <thread>
#include <chrono>

namespace eacham
{

class OptimizerPipeline : public IPipeline
{
public:
    OptimizerPipeline(std::unique_ptr<IMapOptimizer> optimizer)
        : optimizer(std::move(optimizer))
        , isNeedOptimize(false)
    {
    }

    void NeedOptimize()
    {
        isNeedOptimize = true;
    }

protected:
    void Process() override
    {
        if (isNeedOptimize)
        {
            isNeedOptimize = false;
            this->optimizer->Optimize();
        }
    }

private:
    std::unique_ptr<IMapOptimizer> optimizer;

    std::atomic<bool> isNeedOptimize;


};

}