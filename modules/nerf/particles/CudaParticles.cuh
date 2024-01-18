#pragma once

#include <curand_kernel.h>

struct ParticlesConfig
{
    const uint numParticles;
    const float particleRaius;
    const uint dataSize;
};

class CudaParticles
{
private:
    struct cudaGraphicsResource* cudaVbo;
    curandState *d_rngStates = 0;
    uint numThreads;
    uint numBlocks;

    const ParticlesConfig config;

public:
    CudaParticles(const ParticlesConfig& config)
        : config{config}
        {}

public:
    void Initialize(const GLuint& vbo);

    void Call(const float dt);
};