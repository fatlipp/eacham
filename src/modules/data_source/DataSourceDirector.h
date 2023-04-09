#pragma once

#include <vector>
#include <list>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include "data_source/IDataSource.h"
#include "data_source/dataset/DataSourceRgbdTum.h"
#include "data_source/dataset/DataSourceKittyStereo.h"
#include "data_source/dataset/DataSourceKittyLidar.h"

#include "config/Config.h"
#include "types/DataTypes.h"
#include "tools/Tools3d.h"

namespace eacham
{

template<typename T>
class DataSourceDirector
{
public:
    std::unique_ptr<IDataSource<stereodata_t>> Build(const ConfigSource& config)
    {
        if (config.type == DataSourceType::DATASET)
        {
            switch (config.configDataset.type)
            {
                case DatasetType::TUM_RGBD:
                    return std::make_unique<DataSourceRgbdTum<T>>(config.configDataset.path);
                case DatasetType::KITTI_STEREO:
                    return std::make_unique<DataSourceKittyStereo<T>>(config.configDataset.path);
            }
            
            return std::make_unique<DataSourceKittyLidar<T>>(config.configDataset.path);
        }

        return nullptr;
    }
};

} // namespace eacham