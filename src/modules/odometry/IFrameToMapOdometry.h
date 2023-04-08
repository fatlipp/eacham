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
#include "map/LocalMap.h"

namespace eacham
{
template<typename T>
class IFrameToMapOdometry : public IOdometry<T>
{
public:
    IFrameToMapOdometry()
    {
        this->localMap = std::make_unique<LocalMap>(10U);
    }

public:
    const LocalMap* const GetLocalMap() const
    {
        return localMap.get();
    }

public:
    std::unique_ptr<LocalMap> localMap;
};

} // namespace eacham