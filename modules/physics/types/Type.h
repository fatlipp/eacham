#pragma once

#include <string>
// #include <thrust/device_ptr.h>
#include <vector_types.h>

struct Vec3
{
    float x;
    float y;
    float z;
};

enum class ObjectType
{
    Particle,
    Cube2d2,
    Cube2d3,
    Cube3d2
};

struct Particle
{
    float3 position;
    float3 velocity;
};

struct Spring
{
    uint id1;
    uint id2;
    float k;
    float length;
    float damping;
};

struct MetaData
{
    uint id;
    float radius;
    float mass;
    float bounce;
    float damping;
    float tanVel;
};

struct Command
{
    std::string type;
    uint index;
    Vec3 pos;
};