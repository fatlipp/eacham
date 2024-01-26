#include <cuda/Operators.cuh>
#include <soft/CudaRigid.cuh>

void SoftCube2d2::Init(const uint startId, const float radius, const float3 offset)
{
    /*
    0 - 1
    | X |
    3 - 2
    */
    const float len = springs[0]->length;
    const float lenDiag = std::sqrt(len * len + len * len);

    const uint idd = startId;
    springs[0]->id1 = idd + 0;
    springs[0]->id2 = idd + 1;

    springs[1]->id1 = idd + 1;
    springs[1]->id2 = idd + 2;

    springs[2]->id1 = idd + 2;
    springs[2]->id2 = idd + 3;

    springs[3]->id1 = idd + 3;
    springs[3]->id2 = idd + 0;
    
    springs[4]->id1 = idd + 0;
    springs[4]->id2 = idd + 2;
    springs[4]->length = lenDiag;

    springs[5]->id1 = idd + 1;
    springs[5]->id2 = idd + 3;
    springs[5]->length = lenDiag;

    particles[0]->position = offset + float3{0.0f, 0, 0};
    particles[1]->position = offset + float3{len, 0, 0};
    particles[2]->position = offset + float3{len, len, 0};
    particles[3]->position = offset + float3{0.0f, len, 0};

    particles[0]->velocity = float3{0.0f, 0, 0};
    particles[1]->velocity = float3{0.0f, 0, 0};
    particles[2]->velocity = float3{0.0f, 0, 0};
    particles[3]->velocity = float3{0.0f, 0, 0};
}

void SoftCube2d3::Init(const uint startId, const float radius, const float3 offset)
{
    /*
    0 -1 - 2
    |  /\ |
    7  +  3
    |  \/ |
    6 -5- 4
    */

    const float len = springs[0]->length;
    const float lenDiag = std::sqrt(len * len + len * len);

    for (int i = 0; i < 8; ++i)
    {
        const uint idd = startId + i;
        springs[i]->id1 = idd;
        springs[i]->id2 = i == 7 ? startId : idd + 1;
    }
    
    const uint idd = startId;
    springs[8]->id1 = idd + 1;
    springs[8]->id2 = idd + 3;
    springs[8]->length = lenDiag;
    
    springs[9]->id1 = idd + 3;
    springs[9]->id2 = idd + 5;
    springs[9]->length = lenDiag;
    
    springs[10]->id1 = idd + 5;
    springs[10]->id2 = idd + 7;
    springs[10]->length = lenDiag;
    
    springs[11]->id1 = idd + 7;
    springs[11]->id2 = idd + 1;
    springs[11]->length = lenDiag;
    
    springs[12]->id1 = idd + 1;
    springs[12]->id2 = idd + 5;
    springs[12]->length = len * 2;
    
    springs[13]->id1 = idd + 3;
    springs[13]->id2 = idd + 7;
    springs[13]->length = len * 2;

    particles[0]->position = offset + float3{0.0f, 0, 0};
    particles[1]->position = offset + float3{len, 0, 0};
    particles[2]->position = offset + float3{len * 2, 0, 0};
    particles[3]->position = offset + float3{len * 2, len, 0};
    particles[4]->position = offset + float3{len * 2, len * 2, 0};
    particles[5]->position = offset + float3{len, len * 2, 0};
    particles[6]->position = offset + float3{0.0f, len * 2, 0};
    particles[7]->position = offset + float3{0.0f, len, 0};

    for (int i = 0; i < 8; ++i)
        particles[i]->velocity = float3{0.0f, 0, 0};
}