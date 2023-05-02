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

    void SetOnStart(std::function<void()> onStartInp)
    {
        this->onStart = onStartInp;
    }

    void SetOnComplete(std::function<void()> onCompleteInp)
    {
        this->onComplete = onCompleteInp;
    }

    virtual bool Optimize() = 0;

protected:
    IMap* map;
    ConfigMapOptimizer config;

    std::function<void()> onStart;
    std::function<void()> onComplete;
};

}