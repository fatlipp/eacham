#pragma once

#include "cuda/Operators.cuh"

#include <curand_kernel.h>

__global__ void initRNG(curandState *const rngStates, const unsigned int seed) 
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;

    curand_init(seed, tid, 0, &rngStates[tid]);
}

__global__ void initialize(float* posData, const uint count, 
    curandState *const rngStates, const uint size)
{
    const uint index = blockIdx.x * blockDim.x + threadIdx.x;

    if (index >= count) return;

    const uint pointStartId1 = index * size;

    curandState localState = rngStates[index];

    // vel
    const float myrandVelX = curand_uniform(&localState) - 0.5f;
    const float myrandVelY = curand_uniform(&localState) - 0.5f;
    const float myrandVelZ = curand_uniform(&localState) - 0.5f;
    posData[pointStartId1 + 3] = std::abs(myrandVelX) <= 0.01 ? 0.01 : myrandVelX;
    posData[pointStartId1 + 4] = std::abs(myrandVelY) <= 0.01 ? 0.01 : myrandVelY;
    posData[pointStartId1 + 5] = 0;//std::abs(myrandVelZ) <= 0.01 ? 0.01 : myrandVelZ;

    // pos
    const float myrandX = curand_uniform(&localState) - 0.5f;
    const float myrandY = curand_uniform(&localState) - 0.5f;
    const float myrandZ = curand_uniform(&localState) - 0.5f;
    posData[pointStartId1 + 0] = myrandX * 5.5;
    posData[pointStartId1 + 1] = myrandY * 5.5;
    posData[pointStartId1 + 2] = myrandZ * 5.5;
}

__global__ void update(float* posData, const uint count, 
    const float dt, const float radius, const uint size)
{
    const uint index = blockIdx.x * blockDim.x + threadIdx.x;

    if (index >= count) return;

    const uint pointStartId1 = index * size;

    float3 pos1 {
        posData[pointStartId1 + 0], 
        posData[pointStartId1 + 1], 
        posData[pointStartId1 + 2] };
    float3 vel1 {
        posData[pointStartId1 + 3], 
        posData[pointStartId1 + 4], 
        posData[pointStartId1 + 5] };

    const float gravity = -0.087f;
    const float bound = 3.0f;
    const float maxDist = (radius + radius);

    const float STEP = 0.1f;
    const float FORCE = 5.f;
    const float FORCE_2 = 1.1f;

    for (int i = 0; i < count; ++i)
    {
        const uint index2 = i * size;
        if (index2 != pointStartId1)
        {
            float3 pos2 = {posData[index2 + 0], posData[index2 + 1], posData[index2 + 2]};
            float3 relPos = pos2 - pos1;

            float distance = length(relPos);

            if (distance <= maxDist)
            {
                float3 vel2 = {posData[index2 + 3], posData[index2 + 4], posData[index2 + 5]};

                float3 normal = normalize(relPos);
                float3 relVel = vel2 - vel1;

                float3 impulse = normal * dot(relVel, normal) * STEP * FORCE;
                vel1 += impulse;
                vel2 -= impulse;

                float3 repulsion = normal * (maxDist - distance) * STEP * FORCE_2;
                pos1 -= repulsion;
                pos2 += repulsion;

                posData[index2 + 0] = pos2.x;
                posData[index2 + 1] = pos2.y;
                posData[index2 + 2] = pos2.z;
                posData[index2 + 3] = vel2.x;
                posData[index2 + 4] = vel2.y;
                posData[index2 + 5] = vel2.z;
            }
        }
    }

    vel1 += float3{0, gravity * STEP, 0};
    vel1 = clamp(vel1, -3, 3);
    pos1 += vel1 * STEP;

    if (pos1.x <= -bound + radius) vel1.x *= -1;
    if (pos1.y <= -bound + radius) vel1.y *= -1;
    if (pos1.z <= -bound + radius) vel1.z *= -1;
    if (pos1.x >= bound - radius) vel1.x *= -1;
    if (pos1.y >= bound - radius) vel1.y *= -1;
    if (pos1.z >= bound - radius) vel1.z *= -1;
    pos1 = clamp(pos1, -bound + radius, bound - radius);

    posData[pointStartId1 + 0] = pos1.x;
    posData[pointStartId1 + 1] = pos1.y;
    posData[pointStartId1 + 2] = pos1.z;
    posData[pointStartId1 + 3] = vel1.x;
    posData[pointStartId1 + 4] = vel1.y;
    posData[pointStartId1 + 5] = vel1.z;
}