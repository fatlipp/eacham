#pragma once

#include <iostream>
#include <memory>

#include "DataSourceTypes.h"
#include "DataSourceKittyStereo.h"
#include "types/DataTypes.h"

namespace eacham
{
template<typename T>
std::unique_ptr<IDataSourceCamera<T>> CreateStereo(const DataSourceType& dataSourceType, const std::string &path)
{
    switch (dataSourceType)
    {
    case DataSourceType::DATASET:
        return std::make_unique<DataSourceKittyStereo<T>>(path);
    }

    return {};
}
    
} // namespace eacham