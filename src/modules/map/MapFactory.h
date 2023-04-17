#pragma once

#include "config/Config.h"
#include "map/IMap.h"
#include "map/LocalMap.h"

namespace eacham
{

class MapFactory
{
public:
    static std::unique_ptr<IMap> Build(const Config& config)
    {
        return std::make_unique<LocalMap>(config.GetGeneral().mapCapacity);
    }
};

} // namespace eacham