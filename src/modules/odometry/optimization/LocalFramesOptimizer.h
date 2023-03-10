#pragma once

#include "odometry/map/LocalMap.h"

namespace odometry
{

class LocalFramesOptimizer
{
public:
    LocalFramesOptimizer();

    bool Optimize(LocalMap &map);
};

}