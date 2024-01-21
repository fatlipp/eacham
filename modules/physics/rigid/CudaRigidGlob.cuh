#pragma once

#include "cuda/Operators.cuh"

#include <curand_kernel.h>
#include <thrust/device_ptr.h>

namespace rigid
{

__global__ void updateVBO(float3* position, const uint startId, const uint count, 
    float3* vbo, const uint countVbo)
{
    const uint id = startId + blockIdx.x * blockDim.x + threadIdx.x;
    

    if (id >= count) return;
    
    // #pragma unroll
    vbo[id - startId] = position[id];
}

__global__ void initRNG(curandState *const rngStates, const unsigned int seed) 
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;

    curand_init(seed, tid, 0, &rngStates[tid]);
}

template<typename T>
__global__ void initializeObjectsDynamics(T* objects, const uint DATA_SIZE, const uint startId, const uint count,
    float3* position, float3* velocity)
{
    const uint id = blockIdx.x * blockDim.x + threadIdx.x;

    if (id >= count) return;
    
    for (int i = 0; i < DATA_SIZE; ++i)
    {
        objects[id].position[i] = &position[(startId + id) * DATA_SIZE + i];
        objects[id].velocity[i] = &velocity[(startId + id) * DATA_SIZE + i];
    }
}

__global__ void update(float3* position, float3* velocity, const uint count, 
    const float radius, const float dt)
{
    const uint id = blockIdx.x * blockDim.x + threadIdx.x;

    if (id >= count) return;

    const float3 gravity {0, -0.001f, 0};
    const float damping = 0.999f;
    const float bound = 4.0f;

    float3& pos = position[id];
    float3& vel = velocity[id];

    vel += gravity * dt;
    pos += vel * dt;

    if (pos.x - radius <= -bound) 
    { 
        pos.x = -bound + radius;
        vel.x *= -damping;
    }
    else if (pos.x + radius >= bound) 
    {
        pos.x = bound - radius;
        vel.x *= -damping;
    }

    if (pos.y - radius <= -bound) 
    { 
        pos.y = -bound + radius;
        vel.y *= -damping;
    }
    // else if (pos.y + radius >= bound) 
    // {
    //     pos.y = bound - radius;
    //     vel.y *= -damping;
    // }
    
    if (pos.z - radius <= -bound) 
    { 
        pos.z = -bound + radius;
        vel.z *= -damping;
    }
    else if (pos.z + radius >= bound) 
    {
        pos.z = bound - radius;
        vel.z *= -damping;
    }

    vel *= 0.996f;
}

template<typename T>
__global__ void updateSpring(T* objects, const uint count,
    const float dt)
{
    const uint id = blockIdx.x * blockDim.x + threadIdx.x;

    if (id >= count) return;

    T& obj = objects[id];

    // printf("%d) count: %lu\n", id, sizeof(obj.springs) / sizeof(Spring));

    for (int i = 0; i < sizeof(obj.springs) / sizeof(Spring); ++i)
    {
        Spring& spring = obj.springs[i];

        const float3& pos1 = *obj.position[spring.id1];
        const float3& pos2 = *obj.position[spring.id2];

        float3& vel1 = *obj.velocity[spring.id1];
        float3& vel2 = *obj.velocity[spring.id2];

        const float3 relPos = pos2 - pos1;
        const float distance = length(relPos);

        const float distention = distance - spring.length;
        const float restorativeForce = -spring.stiffness * distention;// * STEP * FORCE_1; 
        // F = -kx

        float3 relPosNorm;

        if (distance < 0.00001)
        {
            relPosNorm = {0.9, -1.5, 0};
            printf("%d) dist: %f, distention: %f, force: %f\n", 
                id, distance, distention, restorativeForce);
        }
        else
        {
            relPosNorm = normalize(relPos);
        }

        float3 f = relPosNorm * restorativeForce * dt;
        vel1 += (f * -1.0f);
        vel2 += f;
        
        float3 relVel = vel1 - vel2;
        float3 dampingForce = relVel * spring.damping * dt;// * STEP * FORCE_1;
        vel1 += (dampingForce * -1.0f);
        vel2 += dampingForce;
    }
}

__global__ void updateCollisions(float3* positions, float3* velocities, float3* velocitiesOut, 
    const uint count, const float radius, const float dt)
{
    const uint id = blockIdx.x * blockDim.x + threadIdx.x;

    if (id >= count) return;

    const float3& pos1 = positions[id];
    const float3& vel1 = velocities[id];
    float3 force {0, 0, 0};

    const float maxDist = (radius + radius);

    for (int j = 0; j < count; ++j)
    {
        if (id != j)
        {
            const float3& pos2 = positions[j];
            const float3& vel2 = velocities[j];

            float3 relPos = pos2 - pos1;

            float distance = length(relPos);

            if (distance <= maxDist)
            {
                float3 normal = normalize(relPos);

                if (distance < 0.001f)
                {
                    printf("collision zero: %d -> %d\n", id, j);
                    normal = {0.01, 0.01, 0};
                    // continue;
                }

                // spring force
                force += -0.5f * (maxDist - distance) * normal;

                // relative velocity (damping)
                float3 relVel = vel2 - vel1;
                force += 0.02f * relVel;

                // relative tangential velocity
                float3 tanVel = relVel - (dot(relVel, normal) * normal);
                force += 0.1f * tanVel;
            }
        }
    }

    velocitiesOut[id] = vel1;
    velocitiesOut[id] += force;
}

}