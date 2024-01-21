#pragma once

#include <rigid/CudaRigid.cuh>
#include <chrono>
#include <thread>
#include <iostream>

namespace forms
{

void InitCube4(SoftCube2* cube, const float radius, const float3 offset)
{
    /*
    0 - 1
    | X |
    3 - 2
    */
    float len = 0.1f + radius * 1.5f;
    float lenDiag = std::sqrt(len * len + len * len);

    for (int i = 0; i < 6; ++i)
    {
        cube->springs[i].stiffness = 0.035f;
        cube->springs[i].damping = 0.028f;
    }

    cube->springs[0].id1 = 0;
    cube->springs[0].id2 = 1;
    cube->springs[0].length = len;
    
    cube->springs[1].id1 = 0;
    cube->springs[1].id2 = 2;
    cube->springs[1].length = lenDiag;

    cube->springs[2].id1 = 0;
    cube->springs[2].id2 = 3;
    cube->springs[2].length = len;

    cube->springs[3].id1 = 1;
    cube->springs[3].id2 = 2;
    cube->springs[3].length = len;

    cube->springs[4].id1 = 1;
    cube->springs[4].id2 = 3;
    cube->springs[4].length = lenDiag;

    cube->springs[5].id1 = 2;
    cube->springs[5].id2 = 3;
    cube->springs[5].length = len;

    *cube->position[0] = offset + float3{0.0f, 0, 0};
    *cube->position[1] = offset + float3{radius, 0, 0};
    *cube->position[2] = offset + float3{radius, radius, 0};
    *cube->position[3] = offset + float3{0.0f, radius, 0};

    *cube->velocity[0] = float3{0.0f, 0, 0};
    *cube->velocity[1] = float3{0.0f, 0, 0};
    *cube->velocity[2] = float3{0.0f, 0, 0};
    *cube->velocity[3] = float3{0.0f, 0, 0};
}

void InitCube8(SoftCube3* cube, const float radius, const float3 offset)
{
    /*
    0 -1 - 2
    |  /\ |
    7  +  3
    |  \/ |
    6 -5- 4
    */
    for (int i = 0; i < 12; ++i)
    {
        cube->springs[i].stiffness = 0.335f;
        cube->springs[i].damping = 0.128f;
    }

    float len = 0.1f + radius * 1.5f;
    float lenDiag = std::sqrt(len * len + len * len);

    for (int i = 0; i < 8; ++i)
    {
        cube->springs[i].id1 = i;
        cube->springs[i].id2 = i == 7 ? 0 : i + 1;
        cube->springs[i].length = len;
    }
    
    cube->springs[8].id1 = 1;
    cube->springs[8].id2 = 3;
    cube->springs[8].length = lenDiag;
    
    cube->springs[9].id1 = 3;
    cube->springs[9].id2 = 5;
    cube->springs[9].length = lenDiag;
    
    cube->springs[10].id1 = 5;
    cube->springs[10].id2 = 7;
    cube->springs[10].length = lenDiag;
    
    cube->springs[11].id1 = 7;
    cube->springs[11].id2 = 1;
    cube->springs[11].length = lenDiag;
    
    cube->springs[12].id1 = 1;
    cube->springs[12].id2 = 5;
    cube->springs[12].length = len * 2;
    cube->springs[12].stiffness = 0.135f;
    cube->springs[12].damping = 0.028f;
    
    cube->springs[13].id1 = 3;
    cube->springs[13].id2 = 7;
    cube->springs[13].length = len * 2;
    cube->springs[13].stiffness = 0.335f;
    cube->springs[13].damping = 0.128f;

    *cube->position[0] = offset + float3{0.0f, 0, 0};
    *cube->position[1] = offset + float3{radius, 0, 0};
    *cube->position[2] = offset + float3{radius * 2, 0, 0};
    *cube->position[3] = offset + float3{radius * 2, radius, 0};
    *cube->position[4] = offset + float3{radius * 2, radius * 2, 0};
    *cube->position[5] = offset + float3{radius, radius * 2, 0};
    *cube->position[6] = offset + float3{0.0f, radius * 2, 0};
    *cube->position[7] = offset + float3{0.0f, radius, 0};

    *cube->velocity[0] = float3{0.0f, 0, 0};
    *cube->velocity[1] = float3{0.0f, 0, 0};
    *cube->velocity[2] = float3{0.0f, 0, 0};
    *cube->velocity[3] = float3{0.0f, 0, 0};
    *cube->velocity[4] = float3{0.0f, 0, 0};
    *cube->velocity[5] = float3{0.0f, 0, 0};
    *cube->velocity[6] = float3{0.0f, 0, 0};
    *cube->velocity[7] = float3{0.0f, 0, 0};
}

template<int SIZE>
void InitLine(SoftLine<SIZE>* line, const float radius, const float3 offset)
{
    /*
    0 -1 - 2-...-[SIZE-1]
    */
    float len = radius * 2.0f;
    float lenDiag = std::sqrt(len * len + len * len);
    for (int i = 0; i < SIZE - 1; ++i)
    {
        line->springs[i].stiffness = 0.335f;
        line->springs[i].damping = 0.128f;
        line->springs[i].id1 = i;
        line->springs[i].id2 = i + 1;
        line->springs[i].length = len;
    }
        
    for (int i = 0; i < SIZE; ++i)
    {
        *line->position[i] = offset + float3{i * len, i * i * 0.01f, 0};
        *line->velocity[i] = float3{0.0f, 0, 0};
    }
}

}