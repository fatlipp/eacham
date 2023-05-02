#pragma once

#include <vector>
#include <list>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include "IOdometry.h"
#include "types/DataTypes.h"
#include "data_source/IDataSourceCamera.h"
#include "frame/IFrameCreator.h"
#include "motion_estimator/IMotionEstimator.h"
#include "map/IMap.h"
#include "map/LocalMap.h"

namespace eacham
{
template<typename T>
class IVisualOdometry : public IOdometry<T>
{
public:
    IVisualOdometry()
        : isMapOptimizationProcess{false}
    {
    }

public:
    void SetFrameCreator(std::unique_ptr<IFrameCreator> frameCreatorInp)
    {
        this->frameCreator = std::move(frameCreatorInp);
    }

    void SetMotionEstimator(std::unique_ptr<IMotionEstimator> motionEstimatorInp)
    {
        this->motionEstimator = std::move(motionEstimatorInp);
    }

    void SetMap(IMap* map)
    {
        std::lock_guard<std::mutex> lock(this->mapMutex);

        this->map = map;
    }

    void SetUpdatseStarted()
    {
        this->isMapOptimizationProcess = true;
    }

    void SetUpdatseCompleted()
    {
        this->isMapOptimizationProcess = false;
    }

public:
    // const IFrame* const GetLastFrame() const
    // {
    //     std::lock_guard<std::mutex> lock(this->mapMutex);

    //     if (this->map->GetSize() > 0)
    //     {
    //         return &this->map->GetFrames().back();
    //     }

    //     return nullptr;
    // }
    IFrame GetLastFrame() const
    {
        std::lock_guard<std::mutex> lock(this->mapMutex);

        if (this->map->GetSize() > 0)
        {
            return this->map->GetFrames().back();
        }

        return {};
    }

    // const IMap* const GetMap() const
    // {
    //     std::lock_guard<std::mutex> lock(this->mapMutex);

    //     return this->map;
    // }

protected:
    void WaitForLocalMap() const
    {
        while (this->isMapOptimizationProcess)
        {
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

protected:
    std::unique_ptr<IFrameCreator> frameCreator;
    std::unique_ptr<IMotionEstimator> motionEstimator;
    IMap* map;

    std::atomic<bool> isMapOptimizationProcess;

private:
    mutable std::mutex mapMutex;

};

} // namespace eacham