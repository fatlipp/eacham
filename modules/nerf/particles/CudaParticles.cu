#include "cuda/Types.h"
#include "cuda/CudaHelper.h"

#include "particles/CudaParticles.cuh"
#include "particles/CudaParticlesGlob.cuh"

#include <iostream>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

void CudaParticles::Initialize(const GLuint& vbo)
{
    cuda::registerGLBufferObject(vbo, &cudaVbo);
    cuda::computeGridSize(config.numParticles, 64, numBlocks, numThreads);

    cudaMalloc((void **)&d_rngStates, numBlocks * numThreads * sizeof(curandState));
    initRNG<<<numBlocks, numThreads>>>(d_rngStates, 1000);

    // auto mapAndCall = [&cudaVbo](){
    //     float* dPos = (float *)cuda::mapGLBufferObject(&cudaVbo);
    //     initialize<<<numBlocks, numThreads>>>(dPos, config.numParticles, 
    //         d_rngStates, config.dataSize);
    //     cuda::unmapGLBufferObject(cudaVbo);
    // };

    float* dPos = (float *)cuda::mapGLBufferObject(&cudaVbo);
    initialize<<<numBlocks, numThreads>>>(dPos, config.numParticles, 
        d_rngStates, config.dataSize);
    cuda::unmapGLBufferObject(cudaVbo);
}

void CudaParticles::Call(const float dt) 
{
    float* dPos = (float *)cuda::mapGLBufferObject(&cudaVbo);
    update<<<numBlocks, numThreads>>>(dPos, config.numParticles, 
        dt, config.particleRaius, config.dataSize);

    GET_CUDA_ERROR("Kernel execution failed");

    cuda::unmapGLBufferObject(cudaVbo);
}