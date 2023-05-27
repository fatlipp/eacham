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
        , isProcess(false)
    {
        this->optimizer->SetOnComplete([&](){
            isProcess = false;
        });
    }

    void NeedOptimize()
    {
        if (!isNeedOptimize)
        {
            isNeedOptimize = true;
        }
    }

protected:
    void Process() override
    {
        if (isNeedOptimize && !isProcess)
        {
            this->optimizer->Optimize();
            isNeedOptimize = false;
        }
    }

private:
    std::unique_ptr<IMapOptimizer> optimizer;

    std::atomic<bool> isNeedOptimize;
    std::atomic<bool> isProcess;


};

}