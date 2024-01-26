#pragma once

#include "config/SceneConfig.h"

#include "types/Type.h"
#include <thrust/device_ptr.h>

template<int SIZE, int SPRINGS>
struct ISoftObject
{
    Spring* springs[SPRINGS];
    Particle* particles[SIZE];

    static inline int GetParticlesCount() noexcept { return SIZE; }
    static inline int GetSpringsCount() noexcept { return SPRINGS; }
};

struct SoftCube2d2 : public ISoftObject<4, 6>
{
    void Init(const uint startId, const float radius, const float3 offset);
};

struct SoftCube2d3 : public ISoftObject<8, 14>
{
    void Init(const uint startId, const float radius, const float3 offset);
};


struct SoftCube3d2 : public ISoftObject<8, 20>
{
    void Init(const uint startId, const float radius, const float3 offset);
};

static uint GetConfigParticlesCount(const ObjectConfig& config)
{
    if (config.type == ObjectType::Particle) return config.count;
    if (config.type == ObjectType::Cube2d2) return config.count * 4;
    if (config.type == ObjectType::Cube2d3) return config.count * 8;
    if (config.type == ObjectType::Cube3d2) return config.count * 8;
    return config.count;
}

static uint GetConfigSpringsCount(const ObjectConfig& config)
{
    if (config.type == ObjectType::Particle) return 0;
    if (config.type == ObjectType::Cube2d2) return config.count * 6;
    if (config.type == ObjectType::Cube2d3) return config.count * 14;
    if (config.type == ObjectType::Cube3d2) return config.count * 20;
    return config.count - 1;
}
