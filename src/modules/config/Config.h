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
    {DatasetType::TUM_RGBD, "TUM_RGBD"},
    {DatasetType::KITTI_STEREO, "KITTI_STEREO"},
})

NLOHMANN_JSON_SERIALIZE_ENUM(SensorType, {
    {SensorType::REALSENSE_RGBD, "REALSENSE_RGBD"},
    {SensorType::REALSENSE_STEREO, "REALSENSE_STEREO"},
    {SensorType::REALSENSE_MONO, "REALSENSE_MONO"},
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

    friend void from_json(const nlohmann::json& j, ConfigGeneral& value)
    {
        j.at("maxFrames").get_to(value.maxFrames);
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

struct ConfigSensor
{
    SensorType type;

    friend void from_json(const nlohmann::json& j, ConfigSensor& value)
    {
        j.at("type").get_to<SensorType>(value.type);
    }
};

struct ConfigSource
{
    DataSourceType type;
    ConfigDataset configDataset;
    ConfigSensor configSensor;

    friend void from_json(const nlohmann::json& j, ConfigSource& value)
    {
        j.at("type").get_to<DataSourceType>(value.type);

        if (value.type == DataSourceType::DATASET)
        {
            j.at("dataset").get_to<ConfigDataset>(value.configDataset);
        }
        else if (value.type == DataSourceType::SENSOR)
        {
            j.at("sensor").get_to<ConfigSensor>(value.configSensor);
        }
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
            this->source = data["source"].get<ConfigSource>();
            this->odometry = data["odometry"].get<ConfigOdometry>();

            // Print the values
            std::cout << "general: " << std::endl;
            std::cout << "-maxFrames: " << general.maxFrames << std::endl;
            
            std::cout << "source: " << std::endl;
            std::cout << "-type: " << static_cast<int>(source.type) << std::endl;
            std::cout << "dataset: " << std::endl;
            std::cout << "-type: " << static_cast<int>(this->source.configDataset.type) << std::endl;
            std::cout << "-path: " << this->source.configDataset.path << std::endl;
            
            std::cout << "odometry: " << std::endl;
            std::cout << "-featureExtractorType: " << static_cast<int>(odometry.featureExtractorType) << std::endl;
            std::cout << "-motionEstimatorType: " << static_cast<int>(odometry.motionEstimatorType) << std::endl;

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

    const ConfigSource& GetSource() const
    {
        return source;
    }

    const ConfigOdometry& GetOdometry() const
    {
        return odometry;
    }

private:
    const std::string confidPath;

    ConfigGeneral general;
    ConfigSource source;
    ConfigOdometry odometry;

};

}