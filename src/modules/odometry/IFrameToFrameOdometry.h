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
class IFrameToFrameOdometry : public IVisualOdometry<T>
{
public:
    IFrameToFrameOdometry()
    {
    }

public:
    void Reset() override
    {
        std::lock_guard<std::mutex> lock(this->syncMutex);
        
        IOdometry<T>::Reset();
        
        // this->lastFrame->Reset();
    }

public:
    const Frame* const GetLastFrame() const
    {
        std::lock_guard<std::mutex> lock(this->syncMutex);
        return &this->lastFrame;
    }

protected:
    Frame lastFrame;

    mutable std::mutex syncMutex;
};

} // namespace eacham