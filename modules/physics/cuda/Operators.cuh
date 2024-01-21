#pragma once

#include <curand_kernel.h>
#include <cuda_gl_interop.h>

inline __device__ float3 operator/(float3 a, float b);

inline __device__ float dot(float3 a, float3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline __device__ float distSqr(float3 a, float3 b)
{
    return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2);
}
inline __device__ float dist(float3 a, float3 b)
{
    return std::sqrt(distSqr(a, b));
}
inline __device__ float length(float3 v)
{
    return std::sqrt(dot(v, v));
}
inline __device__ float normSqr(float3 a)
{
    return a.x * a.x + a.y * a.y + a.z * a.z;
}
inline __device__ float norm(float3 a)
{
    return std::sqrt(normSqr(a));
}
inline __device__ float3 normalize(float3 a)
{
    const auto n = norm(a);

    return {a / n};
}
inline __device__ void clamp(float3& a, float min, float max)
{
    if (a.x <= min) a.x = min;
    if (a.y <= min) a.y = min;
    if (a.z <= min) a.z = min;

    if (a.x >= max) a.x = max;
    if (a.y >= max) a.y = max;
    if (a.z >= max) a.z = max;
}
inline __device__ void clamp01(float3& a)
{
    clamp(a, 0.0f, 1.0f);
}

inline __device__ float3 operator-(float3 a, float3 b)
{
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline __device__ void operator/=(float3 &a, float3 b)
{
    a.x /= b.x;
    a.y /= b.y;
    a.z /= b.z;
}

inline __device__ float3 operator/(float3 a, float b)
{
    return make_float3(a.x / b, a.y / b, a.z / b);
}

inline  __device__ float3 operator*(float3 a, float3 b)
{
    return make_float3(a.x * b.x, a.y * b.y, a.z * b.z);
}

inline  __device__ void operator*=(float3 &a, float3 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
}

inline  __device__ float3 operator*(float3 a, float b)
{
    return make_float3(a.x * b, a.y * b, a.z * b);
}

inline  __device__ float3 operator*(float b, float3 a)
{
    return make_float3(b * a.x, b * a.y, b * a.z);
}

inline  __device__ void operator*=(float3 &a, float b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
}

inline  __device__ int3 operator*(int3 a, int3 b)
{
    return make_int3(a.x * b.x, a.y * b.y, a.z * b.z);
}

inline __device__ void operator+=(float3 &a, float3 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
}

inline __host__ __device__ float3 operator+(float3 a, float3 b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline __device__ float3 operator+(float3 &a, float b)
{
    return {a.x + b, a.y + b, a.z + b};
}

inline __device__ void operator-=(float3 &a, float3 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
}