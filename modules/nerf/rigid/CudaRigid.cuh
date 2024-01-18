#pragma once

#include <curand_kernel.h>

struct RgidConfig
{
    const uint numParticles;
    const float particleRaius;
    const uint dataSize;
};

class CudaRigid
{
private:
    struct cudaGraphicsResource* cudaVbo;
    curandState *d_rngStates = 0;
    uint numThreads;
    uint numBlocks;

    const RgidConfig config;

public:
    CudaRigid(const RgidConfig& config)
        : config{config}
        {}

public:
    void Initialize(const GLuint& vbo);

    void Call(const float dt);
};