#pragma once

#include "odometry/frame/Frame.h"
#include <list>


namespace odometry
{

class LocalFramesOptimizer
{
public:
    LocalFramesOptimizer();

    bool Optimize(const std::list<Frame> &frames);

};

}