#pragma once

#include <iostream>

#include <cuda_gl_interop.h>

namespace cuda
{

bool initCuda();

void registerGLBufferObject(uint vbo, struct cudaGraphicsResource **cuda_vbo_resource);

void* mapGLBufferObject(struct cudaGraphicsResource **cuda_vbo_resource);

void unmapGLBufferObject(struct cudaGraphicsResource *cuda_vbo_resource);

uint iDivUp(uint a, uint b);

void computeGridSize(uint n, uint blockSize, uint &numBlocks,
                     uint &numThreads); 

void getLastCudaError(const char *errorMessage, const char *file,
                               const int line);

template<typename T>
T* extendAllocatedMemory(T* oldMemory, const int oldSize, const int newSize)
{
  T* dataBuf;
  cudaMallocManaged(&dataBuf, newSize * sizeof(T));
  cudaMemcpy(dataBuf, oldMemory, oldSize * sizeof(T), 
    cudaMemcpyKind::cudaMemcpyDefault);

  cudaFree(oldMemory);
  
  return dataBuf;
}

}

#define GET_CUDA_ERROR(msg) cuda::getLastCudaError(msg, __FILE__, __LINE__);