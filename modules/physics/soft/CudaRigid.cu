#include "cuda/CudaHelper.h"
#include "soft/SoftFactory.h"

#include "soft/CudaRigid.cuh"
#include "soft/CudaRigidKernels.cuh"

#include <iostream>

#include <thrust/device_ptr.h>

void CudaRigid::Initialize(const SceneConfig& config)
{
    this->config = config;

    this->particlesCount = 0;
    this->springsCount = 0;
    this->cudaVbos.resize(config.objects.size());

    for (const auto& obj : config.objects)
    {
        particlesCount += GetConfigParticlesCount(obj);
        springsCount += GetConfigSpringsCount(obj);
        numParticlesForVbo.push_back(GetConfigParticlesCount(obj));
    }

    cudaMallocManaged((void **)&particles, particlesCount * sizeof(Particle));
    cudaMallocManaged((void **)&metaData, particlesCount * sizeof(MetaData));
    cudaMallocManaged((void **)&particlesNew, particlesCount * sizeof(Particle));
    cudaMallocManaged((void **)&springs, springsCount * sizeof(Spring));
    
    std::cout << "cuda: particlesCount: " << particlesCount << std::endl;
    std::cout << "cuda: springsCount: " << springsCount << std::endl;
    
    int i = 0;
    for (const auto& obj : config.objects)
    {
        cuda::registerGLBufferObject(obj.vboVertices, &cudaVbos[i++]);
    }

    InitScene(config, particles, metaData, springs);

    GET_CUDA_ERROR("Spring initialization is failed");

    std::cout << "cuda: SUCCESSFULLY initialized." << std::endl;
}

void CudaRigid::Reset() 
{
    InitScene(config, particles, metaData, springs);
}

void CudaRigid::Call(const float dt) 
{
    cudaMemcpy(particlesNew, particles, particlesCount * sizeof(Particle), 
        cudaMemcpyKind::cudaMemcpyDeviceToDevice);

    // 1. update spring
    if (springsCount > 0)
    {
        cuda::computeGridSize(springsCount, 128, numBlocks, numThreads);
        soft::updateSpring<<<numBlocks, numThreads>>>(springs, particles, 
            metaData, particlesNew, springsCount);
        cudaDeviceSynchronize();

        cudaMemcpy(particles, particlesNew, particlesCount * sizeof(Particle), 
            cudaMemcpyKind::cudaMemcpyDeviceToDevice);
    }

    cuda::computeGridSize(particlesCount, 128, numBlocks, numThreads);

    // 2. collisions
    soft::updateCollisions<<<numBlocks, numThreads>>>(particles, metaData,
        particlesNew, particlesCount);
    cudaDeviceSynchronize();
    cudaMemcpy(particles, particlesNew, particlesCount * sizeof(Particle), 
        cudaMemcpyKind::cudaMemcpyDeviceToDevice);

    // 3. update
    soft::update<<<numBlocks, numThreads>>>(config.gravity, config.bound, config.maxSpeed, 
        particles, metaData, particlesCount, dt);
    cudaDeviceSynchronize();

    int prevCount = 0;
    int i = 0;
    for (auto& cudaVbo : cudaVbos)
    {
        float* positionsBuffer = (float*)cuda::mapGLBufferObject(&cudaVbo);
        soft::updateVBO<<<numBlocks, numThreads>>>(particles, prevCount, 
            prevCount + numParticlesForVbo[i], (float3*)positionsBuffer, numParticlesForVbo[i]);
        cuda::unmapGLBufferObject(cudaVbo);

        prevCount += numParticlesForVbo[i];
        ++i;
    }

    GET_CUDA_ERROR("Kernel execution is failed");
}

void CudaRigid::ExtendWalls(const float scale)
{
    this->config.bound *= scale;
}

void CudaRigid::AddObject(const ObjectConfig& obj, const int x, const int y)
{
    uint particlesCountOld = particlesCount;
    uint springsCountOld = springsCount;
    uint itemId = particlesCountOld;

    cudaVbos.push_back({});
    cuda::registerGLBufferObject(obj.vboVertices, &cudaVbos[cudaVbos.size() - 1]);

    particlesCount += GetConfigParticlesCount(obj);
    springsCount += GetConfigSpringsCount(obj);
    numParticlesForVbo.push_back(GetConfigParticlesCount(obj));

    particles = cuda::extendAllocatedMemory<Particle>(particles, particlesCountOld,
        particlesCount);
    metaData = cuda::extendAllocatedMemory<MetaData>(metaData, particlesCountOld,
        particlesCount);
    springs = cuda::extendAllocatedMemory<Spring>(springs, springsCountOld,
        springsCount);

    cudaFree(particlesNew);
    cudaMalloc((void **)&particlesNew, particlesCount * sizeof(Particle));

    float xx = x;
    float yy = y;
    InitObject(obj, particles, metaData, springs, 
        particlesCountOld, springsCountOld, itemId, xx, yy);

    GET_CUDA_ERROR("AddObject is failed");
}