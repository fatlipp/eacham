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

}