#pragma once

#include "DataSourceTypes.h"
#include "core/data_source/ICamera.h"

#include <tuple>
#include <utility>

namespace eacham
{

template<typename... GtT>
class IGroundTruth
{

public:
    using ReturnType = std::tuple<GtT...>; 

public:
    virtual ReturnType Get() const = 0;
};

}