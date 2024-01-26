#pragma once

#include "config/SceneConfig.h"
#include "soft/SoftObjects.h"

#include <curand_kernel.h>
#include <thrust/device_ptr.h>

class CudaRigid
{

public:
    void Initialize(const SceneConfig& config);

    void Reset();
    
    void Call(const float dt);

    void ExtendWalls(const float scale);

    void AddObject(const ObjectConfig& object, const int x, const int y);

public:
    uint GetParticlesCount() const noexcept
    {
        return particlesCount;
    }
    uint GetSpringsCount() const noexcept
    {
        return springsCount;
    }

private:
    SceneConfig config;

    curandState *randomState01 = 0;
    uint numThreads;
    uint numBlocks;
    std::vector<struct cudaGraphicsResource*> cudaVbos;

    Particle* particles;
    Spring* springs;
    MetaData* metaData;
    Particle* particlesNew;

    uint particlesCount;
    uint springsCount;
    std::vector<uint> numParticlesForVbo;
};