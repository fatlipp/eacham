#pragma once


namespace eacham
{

enum class DataSourceType
{
    DATASET,
    SENSOR
};

enum class DatasetType
{
    TUM_RGBD,
    KITTI_STEREO,
};

enum class SensorType
{
    REALSENSE_RGBD,
    REALSENSE_MONO,
    REALSENSE_STEREO,
};

}