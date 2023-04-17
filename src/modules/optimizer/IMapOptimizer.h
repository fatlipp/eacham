#pragma once

#include "map/IMap.h"

namespace eacham
{

class IMapOptimizer
{
public:
    void SetMap(IMap* map)
    {
        this->map = map;
    }

    virtual bool Optimize() = 0;

protected:
    IMap* map;
};

}