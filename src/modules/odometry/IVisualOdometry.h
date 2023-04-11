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
#include "optimization/LocalFramesOptimizer.h"
#include "map/IMap.h"
#include "map/LocalMap.h"

namespace eacham
{
template<typename T>
class IVisualOdometry : public IOdometry<T>
{
public:
    IVisualOdometry() = default;

public:
    void SetFrameCreator(std::unique_ptr<IFrameCreator> frameCreatorInp)
    {
        this->frameCreator = std::move(frameCreatorInp);
    }

    void SetMotionEstimator(std::unique_ptr<IMotionEstimator> motionEstimatorInp)
    {
        this->motionEstimator = std::move(motionEstimatorInp);
    }

    void SetLocalOptimizer(std::unique_ptr<LocalFramesOptimizer> optimizerInp)
    {
        this->localOptimizer = std::move(optimizerInp);
    }

protected:
    std::unique_ptr<IFrameCreator> frameCreator;
    std::unique_ptr<IMotionEstimator> motionEstimator;
    std::unique_ptr<LocalFramesOptimizer> localOptimizer;

};

} // namespace eacham