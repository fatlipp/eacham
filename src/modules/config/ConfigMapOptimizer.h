#pragma once

#include <json.hpp>

#include "config/IConfig.h"

namespace eacham
{

class ConfigMapOptimizer : public IConfig
{
public:
    ConfigMapOptimizer()
        : IConfig("mapOptimizer")
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
        j.at("type").get_to(this->type);
        j.at("maxIterations").get_to(this->maxIterations);
        j.at("mapPointsLimit").get_to(this->mapPointsLimit);

        j.at("keyframeNoiseRot").get_to(this->keyframeNoiseRot);
        j.at("keyframeNoisePos").get_to(this->keyframeNoisePos);
        j.at("odomNoiseRot").get_to(this->odomNoiseRot);
        j.at("odomNoisePos").get_to(this->odomNoisePos);
        j.at("measurementNoiseUV").get_to(this->measurementNoiseUV);
        j.at("huberUV").get_to(this->huberUV);
        j.at("measurementNoise3d").get_to(this->measurementNoise3d);
        j.at("huber3d").get_to(this->huber3d);

        return true;
    }

    const unsigned& GetType() const
    {
        return this->type;
    }

    const unsigned& GetMaxIterations() const
    {
        return this->maxIterations;
    }

    const unsigned& GetPointsLimit() const
    {
        return this->mapPointsLimit;
    }

    const float& GetKeyframeNoiseRot() const
    {
        return this->keyframeNoiseRot;
    }

    const float& GetKeyframeNoisePos() const
    {
        return this->keyframeNoisePos;
    }

    const float& GetOdomNoiseRot() const
    {
        return this->odomNoiseRot;
    }

    const float& GetOdomNoisePos() const
    {
        return this->odomNoisePos;
    }

    const float& GetMeasurementNoiseUv() const
    {
        return this->measurementNoiseUV;
    }

    const float& GetHuberUv() const
    {
        return this->huberUV;
    }

    const float& GetMeasurementNoise3d() const
    {
        return this->measurementNoise3d;
    }

    const float& GetHuber3d() const
    {
        return this->huber3d;
    }

private:
    unsigned type;
    unsigned maxIterations;
    unsigned mapPointsLimit;
    //noise
    float keyframeNoiseRot;
    float keyframeNoisePos;
    float odomNoiseRot;
    float odomNoisePos;
    float measurementNoiseUV;
    float huberUV;
    float measurementNoise3d;
    float huber3d;
};

}