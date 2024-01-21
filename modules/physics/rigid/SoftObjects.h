#pragma once

#include <thrust/device_ptr.h>

struct Spring
{
    uint id1;
    uint id2;
    float stiffness;
    float length;
    float damping;
};

template<int SIZE, int SPRINGS>
struct ISoftObject
{
    Spring springs[SPRINGS];
    float3* position[SIZE];
    float3* velocity[SIZE];
};

struct SoftCube2 : public ISoftObject<4, 6>{};

struct SoftCube3 : public ISoftObject<8, 14>{};

template<int SIZE>
struct SoftLine : public ISoftObject<SIZE, SIZE-1>{};

struct ObjectConfig
{
    uint type;
    float radius;
    GLuint vbo;
    uint count;
};

struct Config
{
    std::vector<ObjectConfig> objects;
};