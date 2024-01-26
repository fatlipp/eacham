#include <cuda/Operators.cuh>
#include <soft/SoftObjects.h>

void SoftCube3d2::Init(const uint startId, const float radius, const float3 offset)
{
    /*
      4  -  5
     /     /
    0 - 1 6
    | X |/
    3 - 2
    */
    const float len = springs[0]->length;
    const float lenDiag = std::sqrt(len * len + len * len);

    const uint idd = startId;
    // foward
    springs[0]->id1 = idd + 0;
    springs[0]->id2 = idd + 1;

    springs[1]->id1 = idd + 1;
    springs[1]->id2 = idd + 2;

    springs[2]->id1 = idd + 2;
    springs[2]->id2 = idd + 3;

    springs[3]->id1 = idd + 3;
    springs[3]->id2 = idd + 0;

    // back
    springs[4]->id1 = idd + 4;
    springs[4]->id2 = idd + 5;

    springs[5]->id1 = idd + 5;
    springs[5]->id2 = idd + 6;

    springs[6]->id1 = idd + 6;
    springs[6]->id2 = idd + 7;

    springs[7]->id1 = idd + 7;
    springs[7]->id2 = idd + 4;

    // diag 1
    springs[8]->id1 = idd + 0;
    springs[8]->id2 = idd + 2;
    springs[8]->length = lenDiag;

    springs[9]->id1 = idd + 1;
    springs[9]->id2 = idd + 3;
    springs[9]->length = lenDiag;
    
    // diag 2
    springs[10]->id1 = idd + 4;
    springs[10]->id2 = idd + 6;
    springs[10]->length = lenDiag;

    springs[11]->id1 = idd + 5;
    springs[11]->id2 = idd + 7;
    springs[11]->length = lenDiag;
    
    // diag 3
    springs[12]->id1 = idd + 0;
    springs[12]->id2 = idd + 5;
    springs[12]->length = lenDiag;

    springs[13]->id1 = idd + 1;
    springs[13]->id2 = idd + 4;
    springs[13]->length = lenDiag;
    
    // diag 4
    springs[14]->id1 = idd + 3;
    springs[14]->id2 = idd + 6;
    springs[14]->length = lenDiag;

    springs[15]->id1 = idd + 2;
    springs[15]->id2 = idd + 7;
    springs[15]->length = lenDiag;
    
    // diag 5
    springs[16]->id1 = idd + 1;
    springs[16]->id2 = idd + 6;
    springs[16]->length = lenDiag;

    springs[17]->id1 = idd + 2;
    springs[17]->id2 = idd + 5;
    springs[17]->length = lenDiag;
    
    // diag 6
    springs[18]->id1 = idd + 0;
    springs[18]->id2 = idd + 7;
    springs[18]->length = lenDiag;

    springs[19]->id1 = idd + 3;
    springs[19]->id2 = idd + 4;
    springs[19]->length = lenDiag;

    const float shift = len;
    particles[0]->position = offset + float3{0.0f, 0, 0};
    particles[1]->position = offset + float3{shift, 0, 0};
    particles[2]->position = offset + float3{shift, shift, 0};
    particles[3]->position = offset + float3{0.0f, shift, 0};
    
    particles[4]->position = offset + float3{0.0f, 0, shift};
    particles[5]->position = offset + float3{shift, 0, shift};
    particles[6]->position = offset + float3{shift, shift, shift};
    particles[7]->position = offset + float3{0.0f, shift, shift};

    for (int i = 0; i < 8; ++i)
        particles[i]->velocity = float3{0.0f, 0, 0};
}

// template<typename T>
// void InitCube3d3(const uint startId, const float radius, const float3 offset)
// {
//     /*  8 -9 - 10
//        /      / |
//       16     17 11  
//      /      /   |
//     0 -1 - 2    12
//     |  /\ |    /
//     7  +  3  18
//     |  \/ | /
//     6 -5- 4
//     */
//     for (int i = 0; i < 14; ++i)
//     {
//         springs[i]->stiffness = 0.335f;
//         springs[i]->damping = 0.128f;
//     }

//     float len = 0.1f + radius * 1.5f;
//     float lenDiag = std::sqrt(len * len + len * len);

//     for (int i = 0; i < 8; ++i)
//     {
//         const uint idd = startId + i;
//         springs[i]->id1 = idd;
//         springs[i]->id2 = i == 7 ? startId : idd + 1;
//         springs[i]->length = len;
//     }
    
//     const uint idd = startId;
//     springs[8]->id1 = idd + 1;
//     springs[8]->id2 = idd + 3;
//     springs[8]->length = lenDiag;
    
//     springs[9]->id1 = idd + 3;
//     springs[9]->id2 = idd + 5;
//     springs[9]->length = lenDiag;
    
//     springs[10]->id1 = idd + 5;
//     springs[10]->id2 = idd + 7;
//     springs[10]->length = lenDiag;
    
//     springs[11]->id1 = idd + 7;
//     springs[11]->id2 = idd + 1;
//     springs[11]->length = lenDiag;
    
//     springs[12]->id1 = idd + 1;
//     springs[12]->id2 = idd + 5;
//     springs[12]->length = len * 2;
//     springs[12]->stiffness = 0.135f;
//     springs[12]->damping = 0.028f;
    
//     springs[13]->id1 = idd + 3;
//     springs[13]->id2 = idd + 7;
//     springs[13]->length = len * 2;
//     springs[13]->stiffness = 0.335f;
//     springs[13]->damping = 0.128f;

//     *position[0] = offset + float3{0.0f, 0, 0};
//     *position[1] = offset + float3{radius, 0, 0};
//     *position[2] = offset + float3{radius * 2, 0, 0};
//     *position[3] = offset + float3{radius * 2, radius, 0};
//     *position[4] = offset + float3{radius * 2, radius * 2, 0};
//     *position[5] = offset + float3{radius, radius * 2, 0};
//     *position[6] = offset + float3{0.0f, radius * 2, 0};
//     *position[7] = offset + float3{0.0f, radius, 0};

//     *velocity[0] = float3{0.0f, 0, 0};
//     *velocity[1] = float3{0.0f, 0, 0};
//     *velocity[2] = float3{0.0f, 0, 0};
//     *velocity[3] = float3{0.0f, 0, 0};
//     *velocity[4] = float3{0.0f, 0, 0};
//     *velocity[5] = float3{0.0f, 0, 0};
//     *velocity[6] = float3{0.0f, 0, 0};
//     *velocity[7] = float3{0.0f, 0, 0};
// }

// }