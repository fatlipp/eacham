#pragma once

#include <iostream>
#include <fstream>

#include <json.hpp>

#include "data_source/DataSourceTypes.h"
#include "motion_estimator/MotionEstimatorType.h"
#include "features/FeatureExtractorType.h"
#include "odometry/OdometryType.h"


namespace eacham
{

NLOHMANN_JSON_SERIALIZE_ENUM(DataSourceType, {
    {DataSourceType::DATASET, "DATASET"},
    {DataSourceType::SENSOR, "SENSOR"},
})

NLOHMANN_JSON_SERIALIZE_ENUM(DatasetType, {
    {DatasetType::TUM, "TUM"},
    {DatasetType::KITTI, "KITTI"},
})

NLOHMANN_JSON_SERIALIZE_ENUM(SensorType, {
    {SensorType::CAMERA, "CAMERA"},
})

NLOHMANN_JSON_SERIALIZE_ENUM(CameraType, {
    {CameraType::RGBD, "RGBD"},
    {CameraType::MONO, "MONO"},
    {CameraType::STEREO, "STEREO"},
})

NLOHMANN_JSON_SERIALIZE_ENUM(FeatureExtractorType, {
    {FeatureExtractorType::ORB, "ORB"},
    {FeatureExtractorType::SIFT, "SIFT"},
    {FeatureExtractorType::SURF, "SURF"},
})

NLOHMANN_JSON_SERIALIZE_ENUM(MotionEstimatorType, {
    {MotionEstimatorType::OPT, "OPT"},
    {MotionEstimatorType::PNP, "PNP"},
})

NLOHMANN_JSON_SERIALIZE_ENUM(OdometryType, {
    {OdometryType::FRAME_TO_FRAME, "F2F"},
    {OdometryType::FRAME_TO_MAP, "F2M"},
    {OdometryType::OPT, "OPT"}
})

}

namespace eacham
{

struct ConfigGeneral
{
    int maxFrames;

    DataSourceType sourceType;
    SensorType sensorType;

    friend void from_json(const nlohmann::json& j, ConfigGeneral& value)
    {
        j.at("maxFrames").get_to(value.maxFrames);
        j.at("source_type").get_to<DataSourceType>(value.sourceType);
        j.at("sensor_type").get_to<SensorType>(value.sensorType);
    }
};

struct ConfigOdometry
{
    FeatureExtractorType featureExtractorType;
    MotionEstimatorType motionEstimatorType;
    OdometryType odometryType;

    friend void from_json(const nlohmann::json& j, ConfigOdometry& value)
    {
        j.at("featureExtractorType").get_to<FeatureExtractorType>(value.featureExtractorType);
        j.at("motionEstimatorType").get_to<MotionEstimatorType>(value.motionEstimatorType);
        j.at("odometryType").get_to<OdometryType>(value.odometryType);
    }
};

struct ConfigDataset
{
    DatasetType type;
    std::string path;

    friend void from_json(const nlohmann::json& j, ConfigDataset& value)
    {
        j.at("path").get_to(value.path);
        j.at("type").get_to<DatasetType>(value.type);
    }
};

struct ConfigCamera
{
    CameraType type;
    // unsigned width;
    // unsigned height;
    // unsigned fx;
    // unsigned fy;
    // unsigned cx;
    // unsigned cy;
    float scale;

    friend void from_json(const nlohmann::json& j, ConfigCamera& value)
    {
        j.at("type").get_to<CameraType>(value.type);
        j.at("scale").get_to(value.scale);
    }
};

class Config
{
public:
    Config(const std::string &confidPath)
        : confidPath(confidPath)
    {
    }

public:
    bool Read()
    {
        std::ifstream configStream(this->confidPath);

        if (configStream.is_open())
        {
            nlohmann::json data = nlohmann::json::parse(configStream);
            this->general = data["general"].get<ConfigGeneral>();
            this->odometry = data["odometry"].get<ConfigOdometry>();

            if (this->general.sourceType == DataSourceType::DATASET)
            {
                this->dataset = data["dataset"].get<ConfigDataset>();
            }
            
            if (this->general.sensorType == SensorType::CAMERA)
            {
                this->camera = data["camera"].get<ConfigCamera>();
            }

            configStream.close();
        }
        else
        {
            std::cout << "Error. Config file is not found: " << this->confidPath << std::endl;

            return false;
        }

        return true;
    }

    const ConfigGeneral& GetGeneral() const
    {
        return general;
    }

    const ConfigOdometry& GetOdometry() const
    {
        return odometry;
    }

    const ConfigDataset& GetDataset() const
    {
        return dataset;
    }

    const ConfigCamera& GetCamera() const
    {
        return camera;
    }

private:
    const std::string confidPath;

    ConfigGeneral general;
    ConfigOdometry odometry;
    ConfigDataset dataset;
    ConfigCamera camera;

};

}