#include "cuda/Types.h"
#include "cuda/CudaHelper.h"

#include "rigid/CudaRigid.cuh"
#include "rigid/CudaRigidGlob.cuh"
#include "rigid/Forms.h"

#include <iostream>

#include <thrust/device_ptr.h>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

CudaRigid::CudaRigid()
{
}

void CudaRigid::Initialize(const Config& config)
{
    cudaVbos.resize(config.objects.size());

    cuda::computeGridSize(1000, 64, numBlocks, numThreads);

    numParticles = 0;
    objectsCount2 = 0;
    objectsCount3 = 0;

    uint pointsCoun = 0;

    for (uint id = 0; const auto& obj : config.objects)
    {
        pointsCoun += obj.count * (obj.type == 0 ? 4 : 8);

        if (obj.type == 0) objectsCount2 += obj.count;
        if (obj.type == 1) objectsCount3 += obj.count;

        numParticlesForVbo.push_back(obj.count * (obj.type == 0 ? 4 : 8));
    }

    cudaMallocManaged((void **)&position, pointsCoun * sizeof(float3));
    cudaMallocManaged((void **)&velocity, pointsCoun * sizeof(float3));
    cudaMallocManaged((void **)&velocityNew, pointsCoun * sizeof(float3));
    cudaMallocManaged((void **)&objects2, objectsCount2 * sizeof(SoftCube2));
    cudaMallocManaged((void **)&objects3, objectsCount3 * sizeof(SoftCube3));

    float x = -2.5;
    float y = 14;

    for (uint id = 0; const auto& obj : config.objects)
    {
        cuda::registerGLBufferObject(obj.vbo, &cudaVbos[id]);

        // objects.push_back(CreateObject<id>());
        if (obj.type == 0)
        {
            for (int i = 0; i < obj.count; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    objects2[i].position[j] = &position[numParticles + i * 4 + j];
                    objects2[i].velocity[j] = &velocity[numParticles + i * 4 + j];
                }
                forms::InitCube4(&objects2[i], obj.radius, {x, y, 0});

                x += obj.radius * 6.f;

                if (x > 2.6)
                {
                    x = -3;
                    y += obj.radius * 6.f;
                }
            }
        }
        else if (obj.type == 1)
        {
            for (int i = 0; i < obj.count; ++i)
            {
                for (int j = 0; j < 8; ++j)
                {
                    objects3[i].position[j] = &position[numParticles + i * 8 + j];
                    objects3[i].velocity[j] = &velocity[numParticles + i * 8 + j];
                }
                forms::InitCube8(&objects3[i], obj.radius, {x, y, 0});

                x += obj.radius * 6.f;

                if (x > 2.6)
                {
                    x = -3;
                    y += obj.radius * 6.f;
                }
            }
        }

        ++id;
        numParticles += obj.count * (obj.type == 0 ? 4 : 8);
    }

    GET_CUDA_ERROR("Spring initialization is failed");
}

void CudaRigid::Call(const float dt) 
{
    // std::cout << "Call: obj2: " << objectsCount2 << ", obj3: " << objectsCount3 << std::endl;
    rigid::update<<<numBlocks, numThreads>>>(position, velocity, numParticles, 0.3f, dt);
    {
        uint numBlocks1, numThreads1;
        cuda::computeGridSize(100, 32, numBlocks1, numThreads1);
        rigid::updateSpring<<<numBlocks1, numThreads1>>>(objects2, objectsCount2, dt);
        rigid::updateSpring<<<numBlocks1, numThreads1>>>(objects3, objectsCount3, dt);
    }

    cudaDeviceSynchronize();
    
    rigid::updateCollisions<<<numBlocks, numThreads>>>(position, velocity, velocityNew, 
        numParticles, 0.3f, dt);
    cudaMemcpy(velocity, velocityNew, numParticles * sizeof(float3), cudaMemcpyKind::cudaMemcpyDeviceToDevice);
    
    cudaDeviceSynchronize();

    int prevCount = 0;
    for (int i = 0; auto& cudaVbo : cudaVbos)
    {
        float* positionsBuffer = (float*)cuda::mapGLBufferObject(&cudaVbo);
        rigid::updateVBO<<<1, numParticlesForVbo[i]>>>(position, prevCount, 
            prevCount + numParticlesForVbo[i], (float3*)positionsBuffer, numParticlesForVbo[i]);
        cuda::unmapGLBufferObject(cudaVbo);

        prevCount += numParticlesForVbo[i];
        ++i;
    }

    GET_CUDA_ERROR("Kernel execution is failed");
}