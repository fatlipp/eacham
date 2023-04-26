#pragma once

#include "map/IMap.h"

#include "config/ConfigMapOptimizer.h"

namespace eacham
{

class IMapOptimizer
{
public:
    IMapOptimizer(const ConfigMapOptimizer& config)
        : config(config)
    {
    }

public:
    void SetMap(IMap* map)
    {
        this->map = map;
    }

    virtual bool Optimize() = 0;

protected:
    IMap* map;
    ConfigMapOptimizer config;
};

}