#pragma once

#include <iostream>
#include <memory>

#include "DataSourceTypes.h"
#include "DataSourceKittyStereo.h"
#include "types/DataTypes.h"

namespace data_source
{
std::unique_ptr<IDataSource<stereodata_t>> CreateStereo(const DataSourceType& dataSourceType, const std::string &path)
{
    switch (dataSourceType)
    {
    case DataSourceType::DATASET:
        return std::make_unique<DataSourceKittyStereo<stereodata_t>>(path);
    }

    return {};
}
    
} // namespace data_source