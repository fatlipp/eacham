#pragma once

#include <vector>
#include <list>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include "data_source/IDataSourceCamera.h"
#include "data_source/dataset/DataSourceRgbdTum.h"
#include "data_source/dataset/DataSourceKittyStereo.h"
#include "data_source/dataset/DataSourceKittyLidar.h"

#ifdef REALSENSE_FOUND
#include "data_source/sensor/CameraRealsenseD435.h"
#endif

#include "config/Config.h"
#include "types/DataTypes.h"
#include "tools/Tools3d.h"

namespace eacham
{

template<typename T>
class VisualDataSourceDirector
{
public:
    std::unique_ptr<IDataSourceCamera<T>> Build(const ConfigSource& config)
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
            // return std::make_unique<DataSourceKittyLidar<T>>(config.configDataset.path);
        }
        // TODO: other devices
        else
        {
#ifdef REALSENSE_FOUND
            return std::make_unique<CameraRealsenseD435<T>>();
#else
            std::cout << "Compiled with no RealSense. Use another device or Dataset" << std::endl;
#endif
        }

        return nullptr;
    }
};

} // namespace eacham