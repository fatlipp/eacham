#pragma once

#include <vector>
#include <list>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include "odometry/IVisualOdometry.h"
#include "types/DataTypes.h"
#include "map/IMap.h"
#include "map/LocalMap.h"

namespace eacham
{
template<typename T>
class IFrameToMapOdometry : public IVisualOdometry<T>
{
public:
    IFrameToMapOdometry()
    {
        this->localMap = std::make_unique<LocalMap>(10U);
    }

public:
    const IMap* const GetLocalMap() const
    {
        std::lock_guard<std::mutex> lock(this->syncMutex);
        return localMap.get();
    }

protected:
    std::unique_ptr<IMap> localMap;

    mutable std::mutex syncMutex;
};

} // namespace eacham