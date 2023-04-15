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
#include "data_source/sensor/CameraRealsenseRgbd.h"
#include "data_source/sensor/CameraRealsenseStereo.h"
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
    std::unique_ptr<IDataSourceCamera<T>> Build(const Config& config)
    {
        if (config.GetGeneral().sensorType != SensorType::CAMERA)
        {
            std::cout << "Non-camera type" << std::endl;
            return nullptr;
        }

        const auto cameraConfig = config.GetCamera();

        std::unique_ptr<IDataSourceCamera<T>> camera;

        if (config.GetGeneral().sourceType == DataSourceType::DATASET)
        {
            const auto dataset = config.GetDataset();

            switch (dataset.type)
            {
                case DatasetType::TUM:
                    camera = (cameraConfig.type == CameraType::RGBD) ? std::make_unique<DataSourceRgbdTum<T>>(dataset.path) : nullptr;
                    break;
                case DatasetType::KITTI:
                    camera = (cameraConfig.type == CameraType::STEREO) ? std::make_unique<DataSourceKittyStereo<T>>(dataset.path) : nullptr;
                    break;
            }
            // return std::make_unique<DataSourceKittyLidar<T>>(dataset.path);
        }
        // TODO: other devices
        else if (config.GetGeneral().sourceType == DataSourceType::SENSOR)
        {
            if (cameraConfig.model == CameraModel::REALSENSE)
            {
#ifdef REALSENSE_FOUND
                switch (cameraConfig.type)
                {
                    case CameraType::RGBD:
                        camera = std::make_unique<CameraRealsenseRgbd<T>>();
                        break;
                    case CameraType::STEREO:
                        camera = std::make_unique<CameraRealsenseStereo<T>>();
                        break;

                }
#else
                std::cout << "Compiled with no RealSense. Use another device or Dataset" << std::endl;
#endif
            }
        }

        if (camera != nullptr)
        {
            camera->Initialize(cameraConfig);
            camera->Start();
        }

        return camera;
    }
};

} // namespace eacham