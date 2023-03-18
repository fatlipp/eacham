#pragma once

#include "odometry/map/LocalMap.h"

namespace eacham
{

class LocalFramesOptimizer
{
public:
    LocalFramesOptimizer();

    bool Optimize(LocalMap &map);
};

}