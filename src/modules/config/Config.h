#pragma once

#include <iostream>
#include <fstream>

#include <json.hpp>

#include "data_source/DataSourceTypes.h"
#include "motion_estimator/MotionEstimatorType.h"
#include "features/FeatureExtractorType.h"

namespace eacham
{

NLOHMANN_JSON_SERIALIZE_ENUM(DataSourceType, {
    {DataSourceType::DATASET, "DATASET"},
    {DataSourceType::REALSENSE, "REALSENSE"},
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

struct ConfigSource
{
    DataSourceType type;
    std::string path;

    friend void from_json(const nlohmann::json& j, ConfigSource& value)
    {
        j.at("type").get_to<DataSourceType>(value.type);
        j.at("path").get_to(value.path);
    }
};

struct ConfigOdometry
{
    FeatureExtractorType featureExtractorType;
    MotionEstimatorType motionEstimatorType;

    friend void from_json(const nlohmann::json& j, ConfigOdometry& value)
    {
        j.at("featureExtractorType").get_to<FeatureExtractorType>(value.featureExtractorType);
        j.at("motionEstimatorType").get_to<MotionEstimatorType>(value.motionEstimatorType);
    }
};

class Config
{
public:
    Config()
    {
    }

public:
    void Read()
    {
        std::ifstream f("/home/blackdyce/Projects/eacham/config/ConfigTUM.json");

        nlohmann::json data = nlohmann::json::parse(f);
        this->general = data["general"].get<ConfigGeneral>();
        this->source = data["source"].get<ConfigSource>();
        this->odometry = data["odometry"].get<ConfigOdometry>();

        // Print the values
        std::cout << "general: " << std::endl;
        std::cout << "-maxFrames: " << general.maxFrames << std::endl;
        
        std::cout << "source: " << std::endl;
        std::cout << "-path: " << source.path << std::endl;
        std::cout << "-type: " << static_cast<int>(source.type) << std::endl;
        
        std::cout << "odometry: " << std::endl;
        std::cout << "-featureExtractorType: " << static_cast<int>(odometry.featureExtractorType) << std::endl;
        std::cout << "-motionEstimatorType: " << static_cast<int>(odometry.motionEstimatorType) << std::endl;
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
    ConfigGeneral general;
    ConfigSource source;
    ConfigOdometry odometry;

};

}