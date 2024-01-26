#pragma once

#include "types/Type.h"
#include "soft/SoftObjects.h"
#include "config/SceneConfig.h"

template<typename T>
void InitCube(const ObjectConfig& obj,
    Particle* particles, MetaData* metaData, Spring* springs,
    uint& particleId, uint& springId, uint& itemId, 
    float& x, float& y)
{
    for (int i = 0; i < obj.count; ++i)
    {
        T cube;
        for (int j = 0; j < T::GetParticlesCount(); ++j)
        {
            cube.particles[j] = &particles[particleId + j];
            metaData[particleId + j].id = itemId;
            metaData[particleId + j].radius = obj.radius;
            metaData[particleId + j].mass = obj.mass;
            metaData[particleId + j].bounce = obj.bounce;
            metaData[particleId + j].damping = obj.damping;
            metaData[particleId + j].tanVel = obj.tanVel;

            if (obj.selfCollision) 
                ++itemId;                 
        }
        for (int j = 0; j < T::GetSpringsCount(); ++j)
        {
            cube.springs[j] = &springs[springId + j];

            cube.springs[j]->k = obj.spring.k;
            cube.springs[j]->damping = obj.spring.damping;

            if (obj.spring.length == 0)
            {
                cube.springs[j]->length = obj.radius * 2.0f + 0.001f; // scale?
            }
            else
            {
                cube.springs[j]->length = obj.spring.length;
            }
        }

        cube.Init(particleId, obj.radius, {x, y, 0});

        particleId += T::GetParticlesCount();
        springId += T::GetSpringsCount();
        ++itemId;

        x += obj.radius * 3.0f;

        if (x > 3)
        {
            x = -3.0f;
            y += obj.radius * 3.0f;
        }
    }
}

void InitObject(const ObjectConfig& obj, 
    Particle* particles, MetaData* metaData, Spring* springs,
    uint& particleId, uint& springId, uint& itemId, 
    float& x, float& y)
{
    switch (obj.type)
    {
    case ObjectType::Particle:
        for (int i = 0; i < obj.count; ++i)
        {
            particles[particleId].position = {x, y, 0};
            particles[particleId].velocity = {0, 0, 0};
            metaData[particleId].id = itemId;
            metaData[particleId].radius = obj.radius;
            metaData[particleId].mass = obj.mass;
            metaData[particleId].bounce = obj.bounce;
            metaData[particleId].damping = obj.damping;
            metaData[particleId].tanVel = obj.tanVel;
            particleId += 1;
            if (obj.selfCollision) 
                ++itemId;

            x += obj.radius * 1.0f;

            if (x > 3)
            {
                x = -3.0f;
                y += obj.radius * 1.0f;
            }
        }
        ++itemId;
        break;
    case ObjectType::Cube2d2:
        InitCube<SoftCube2d2>(obj, particles, metaData, springs, 
            particleId, springId, itemId, x, y);
        break;
    case ObjectType::Cube2d3:
        InitCube<SoftCube2d3>(obj, particles, metaData, springs, 
            particleId, springId, itemId, x, y);
        break;
    case ObjectType::Cube3d2:
        InitCube<SoftCube3d2>(obj, particles, metaData, springs, 
            particleId, springId, itemId, x, y);
        break;
    
    default:
        break;
    }
}

void InitScene(const SceneConfig& config,
    Particle* particles, MetaData* metaData, Spring* springs)
{
    uint itemId = 0;
    uint particleId = 0;
    uint springId = 0;
    float x = 0;
    float y = 5;

    for (const auto& obj : config.objects)
    {
        InitObject(obj, particles, metaData, springs, 
            particleId, springId, itemId, x, y);
    }
}