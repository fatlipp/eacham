#pragma once

#include "cuda/Operators.cuh"

#include <curand_kernel.h>
#include <thrust/device_ptr.h>

namespace soft
{

__global__ void updateVBO(Particle* particles, const uint startId, const uint count, 
    float3* vbo, const uint countVbo)
{
    const uint id = startId + blockIdx.x * blockDim.x + threadIdx.x;

    if (id >= count) return;
    
    // #pragma unroll
    vbo[id - startId] = particles[id].position;
}

__global__ void initRNG(curandState *const rngStates, const unsigned int seed) 
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;

    curand_init(seed, tid, 0, &rngStates[tid]);
}

__global__ void update(const float gravityVel, Bound bound, const float maxSpeed,
    Particle* particles, MetaData* metaData, const uint count,
    const float dt)
{
    const uint id = blockIdx.x * blockDim.x + threadIdx.x;

    if (id >= count) return;

    const float3 gravity {0, -gravityVel, 0};
    const float damping = -0.5f;

    float3& pos = particles[id].position;
    float3& vel = particles[id].velocity;
    float radius = metaData[id].radius;

    vel += (gravity * metaData->mass) * dt;

    if (norm(vel) > maxSpeed)
    {
        vel *= maxSpeed / norm(vel);
    }

    pos += vel * dt;
    vel *= 0.98f;

    if (pos.x - radius <= bound.min.x) 
    { 
        pos.x = bound.min.x + radius;
        vel.x *= damping;
    }
    else if (pos.x + radius >= bound.max.x) 
    {
        pos.x = bound.max.x - radius;
        vel.x *= damping;
    }

    if (pos.y - radius <= bound.min.y)
    { 
        pos.y = bound.min.y + radius;
        vel.y *= damping;
    }
    else if (pos.y + radius >= bound.max.y) 
    {
        pos.y = bound.max.y - radius;
        vel.y *= -damping;
    }
    
    if (pos.z - radius <= bound.min.z) 
    { 
        pos.z = bound.min.z + radius;
        vel.z *= damping;
    }
    else if (pos.z + radius >= bound.max.z) 
    {
        pos.z = bound.max.z - radius;
        vel.z *= damping;
    }

}

__global__ void updateSpring(Spring* springs, Particle* particles, 
    MetaData* metaData, Particle* particlesNew, const uint springsCount)
{
    const uint id = blockIdx.x * blockDim.x + threadIdx.x;

    if (id >= springsCount) return;

    Spring& spring = springs[id];
    const float3& pos1 = particles[spring.id1].position;
    const float3& pos2 = particles[spring.id2].position;
    const float3& vel1 = particles[spring.id1].velocity;
    const float3& vel2 = particles[spring.id2].velocity;

    const float3 relPos = pos2 - pos1;
    const float distance = length(relPos);

    if (distance > 0.00001)
    {
        const float distention = distance - spring.length;
        // F = -kx
        const float restorativeForce = -spring.k * distention;

        float3 relPosNorm = normalize(relPos);

        float3 force = relPosNorm * restorativeForce;
        if (norm(force) > 3.0f)
        {
            printf("high restorative force: %d, force: %f, %f\n", id, force.x, force.y);
            force *= 3.0f / norm(force);
        }
        const float3 f1 = force * -1.0f;// / metaData[spring.id1].mass;
        const float3 f2 = force;// / metaData[spring.id2].mass;

        atomicAdd(&particlesNew[spring.id1].velocity.x, f1.x);
        atomicAdd(&particlesNew[spring.id1].velocity.y, f1.y);
        atomicAdd(&particlesNew[spring.id1].velocity.z, f1.z);
        atomicAdd(&particlesNew[spring.id2].velocity.x, f2.x);
        atomicAdd(&particlesNew[spring.id2].velocity.y, f2.y);
        atomicAdd(&particlesNew[spring.id2].velocity.z, f2.z);
    }
    
    float3 relVel = vel1 - vel2;
    if (length(relVel) > 0.00001)
    {
        float3 force = relVel * spring.damping;

        if (norm(force) > 3.0f)
        {
            printf("high SpringDamping force: %d, force: %f, %f\n", id, force.x, force.y);
            force *= 3.0f / norm(force);
        }

        const float3 f1 = force * -1.0f;// / metaData[spring.id1].mass;
        const float3 f2 = force;// / metaData[spring.id2].mass;

        atomicAdd(&particlesNew[spring.id1].velocity.x, f1.x);
        atomicAdd(&particlesNew[spring.id1].velocity.y, f1.y);
        atomicAdd(&particlesNew[spring.id1].velocity.z, f1.z);
        atomicAdd(&particlesNew[spring.id2].velocity.x, f2.x);
        atomicAdd(&particlesNew[spring.id2].velocity.y, f2.y);
        atomicAdd(&particlesNew[spring.id2].velocity.z, f2.z);
    }
}

__global__ void updateCollisions(Particle* particles, MetaData* metaData,
    Particle* particlesNew, const uint count)
{
    const uint id = blockIdx.x * blockDim.x + threadIdx.x;

    if (id >= count) return;

    const float3& pos1 = particles[id].position;
    const float3& vel1 = particles[id].velocity;
    float3 force {0, 0, 0};

    particlesNew[id].velocity = vel1;

    // use spatial check to improve performance
    for (int j = 0; j < count; ++j)
    {
        if (id != j && metaData[id].id != metaData[j].id)
        {
            const float3& pos2 = particles[j].position;
            const float3& vel2 = particles[j].velocity;

            float3 relPos = pos2 - pos1;

            float distance = length(relPos);

            const float maxDist = (metaData[id].radius + metaData[j].radius);

            if (distance > 0.00001f && distance <= maxDist)
            {
                float massForce = (metaData[j].mass / metaData[id].mass);
                if (massForce < 1.0f) massForce = 1.0f;
                if (massForce > 1.0f) massForce = 3.0f;
                massForce = 1.0f;

                const float3 normal = normalize(relPos);
                force += -metaData[id].bounce * (maxDist - distance) * normal * massForce;

                const float3 relVel = (vel2 - vel1);
                force += metaData[id].damping * relVel;

                const float3 tanVel = relVel - (dot(relVel, normal) * normal);
                force += metaData[id].tanVel * tanVel;
            }
        }
    }
    const float maxForce = 1.5f;
    if (norm(force) > maxForce)
    {
        force *= maxForce / norm(force);
    }

    particlesNew[id].velocity += force / metaData[id].mass;
}

}