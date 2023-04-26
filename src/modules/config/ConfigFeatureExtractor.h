#pragma once

#include <json.hpp>

#include "config/IConfig.h"
#include "features/FeatureExtractorType.h"

namespace eacham
{

NLOHMANN_JSON_SERIALIZE_ENUM(FeatureExtractorType, {
    {FeatureExtractorType::ORB, "ORB"},
    {FeatureExtractorType::SIFT, "SIFT"},
    {FeatureExtractorType::SURF, "SURF"},
})

class ConfigFeatureExtractor : public IConfig
{
public:
    ConfigFeatureExtractor()
        : IConfig("featureExtractor")
        {
        }

public:
    bool Read(const nlohmann::json& data)
    {
        if (!Check(data))
        {
            return false;
        }

        auto j = data[name];
        j.at("type").get_to<FeatureExtractorType>(this->type);
        j.at("maxFeatures").get_to(this->maxFeatures);
        j.at("levelsCount").get_to(this->levelsCount);
        j.at("levelsScale").get_to(this->levelScale);

        return true;
    }

    const FeatureExtractorType& GetType() const
    {
        return type;
    }

    const unsigned& GetMaxFeatures() const
    {
        return maxFeatures;
    }

    const unsigned& GetLevelsCount() const
    {
        return levelsCount;
    }

    const float& GetLevelScale() const
    {
        return levelScale;
    }

private:
    FeatureExtractorType type;
    unsigned maxFeatures;
    unsigned levelsCount;
    float levelScale;

};

}