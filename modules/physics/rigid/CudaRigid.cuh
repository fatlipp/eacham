#pragma once

#include "rigid/SoftObjects.h"

#include <curand_kernel.h>
#include <thrust/device_ptr.h>

class CudaRigid
{
public:
    CudaRigid();

public:
    void Initialize(const Config& config);

    void Call(const float dt);

private:
    Config config;

    curandState *randomState01 = 0;
    uint numThreads;
    uint numBlocks;
    std::vector<struct cudaGraphicsResource*> cudaVbos;

    SoftCube2* objects2;
    SoftCube3* objects3;

    Spring* springs;
    float3* position;
    float3* velocity;
    float3* velocityNew;

    uint objectsCount2;
    uint objectsCount3;
    uint numParticles;
    std::vector<uint> numParticlesForVbo;
};