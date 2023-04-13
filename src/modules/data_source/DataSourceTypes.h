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
    TUM,
    KITTI,
};

enum class SensorType
{
    CAMERA
};

enum class CameraType
{
    RGBD,
    MONO,
    STEREO,
};

}