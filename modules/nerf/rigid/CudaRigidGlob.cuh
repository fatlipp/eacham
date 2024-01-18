#pragma once

#include "cuda/Operators.cuh"

#include <curand_kernel.h>

namespace rigid
{

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
    const float TOL = 0.01f;
    posData[pointStartId1 + 3] = std::abs(myrandVelX) <= TOL ? TOL : myrandVelX;
    posData[pointStartId1 + 4] = std::abs(myrandVelY) <= TOL ? TOL : myrandVelY;
    posData[pointStartId1 + 5] = std::abs(myrandVelZ) <= TOL ? TOL : myrandVelZ;

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

    const float GRAVITY = -0.001;

    float3 pos1 {
        posData[pointStartId1 + 0], 
        posData[pointStartId1 + 1], 
        posData[pointStartId1 + 2] };

    // posData[pointStartId1 + 3] = 0;
    // posData[pointStartId1 + 4] = 0;
    // posData[pointStartId1 + 5] = 0;

    float3 force {0, GRAVITY, 0};

    const float DIST = 1.3f;
    const float TOL = 0.01f;
    const float SPEED = -0.006f;

    for (int i = 0; i < count; ++i)
    {
        if (i == index) continue;
        if (index == 0)
        {
            if (i != 1 && i != 3 && i != 6) continue;
        }
        if (index == 1)
        {
            if (i != 0 && i != 2 && i != 7) continue;
        }
        if (index == 2)
        {
            if (i != 1 && i != 3 && i != 4) continue;
        }
        if (index == 3)
        {
            if (i != 0 && i != 2 && i != 5) continue;
        }
        if (index == 4)
        {
            if (i != 2 && i != 5 && i != 7) continue;
        }
        if (index == 5)
        {
            if (i != 6 && i != 3 && i != 4) continue;
        }
        if (index == 6)
        {
            if (i != 0 && i != 5 && i != 7) continue;
        }
        if (index == 7)
        {
            if (i != 1 && i != 6 && i != 4) continue;
        }

        const uint pointStartId2 = i * size;
        float3 pos2 {
            posData[pointStartId2 + 0], 
            posData[pointStartId2 + 1], 
            posData[pointStartId2 + 2] };

        float3 relPos = pos2 - pos1;
        float dist = norm(relPos);

        float mul = 0.0;
        if (std::abs(dist) > DIST + TOL) mul = -SPEED;
        else if (std::abs(dist) < DIST - TOL) mul = SPEED;

        float3 normal = relPos / dist;
        force = mul * dist * normal;

        posData[pointStartId2 + 3] -= force.x;
        posData[pointStartId2 + 4] -= force.y;
        posData[pointStartId2 + 5] -= force.z;
    }

    posData[pointStartId1 + 3] += force.x;
    posData[pointStartId1 + 4] += force.y;
    posData[pointStartId1 + 5] += force.z;
    float3 vel1 {
        posData[pointStartId1 + 3], 
        posData[pointStartId1 + 4] + GRAVITY, 
        posData[pointStartId1 + 5] };
    pos1 += (vel1 * dt);

    const float pr = radius * 2.0f;
    if (pos1.x > 3 - pr) {pos1.x = 3 - pr; force.x = -force.x * 0.3f; };
    if (pos1.x < -3 + pr) {pos1.x = -3 + pr; force.x = -force.x * 0.3f; };
    if (pos1.y > 3 - pr) {pos1.y = 3 - pr; force.y = -force.y * 0.3f; };
    if (pos1.y < -3 + pr) {pos1.y = -3 + pr; force.y = -force.y * 0.3f; };
    if (pos1.z > 3 - pr) {pos1.z = 3 - pr; force.z = -force.z * 0.3f; };
    if (pos1.z < -3 + pr) {pos1.z = -3 + pr; force.z = -force.z * 0.3f; };

    posData[pointStartId1 + 0] = pos1.x;
    posData[pointStartId1 + 1] = pos1.y;
    posData[pointStartId1 + 2] = pos1.z;
}

}