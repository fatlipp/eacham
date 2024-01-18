#include <iostream>

#include <cuda_gl_interop.h>

namespace cuda
{

bool initCuda() 
{
    int device_count;
    cudaGetDeviceCount(&device_count);

    return device_count > 0;
}

void registerGLBufferObject(uint vbo, struct cudaGraphicsResource **cuda_vbo_resource) 
{
    cudaGraphicsGLRegisterBuffer(cuda_vbo_resource, vbo, cudaGraphicsMapFlagsNone);
}

void* mapGLBufferObject(struct cudaGraphicsResource **cuda_vbo_resource)
{
    void *ptr;
    cudaGraphicsMapResources(1, cuda_vbo_resource, 0);

    size_t num_bytes;
    cudaGraphicsResourceGetMappedPointer((void **)&ptr, &num_bytes, *cuda_vbo_resource);
    
    return ptr;
}

void unmapGLBufferObject(struct cudaGraphicsResource *cuda_vbo_resource) 
{
    cudaGraphicsUnmapResources(1, &cuda_vbo_resource, 0);
}

// Round a / b to nearest higher integer value
uint iDivUp(uint a, uint b) { return (a % b != 0) ? (a / b + 1) : (a / b); }

// compute grid and thread block size for a given number of elements
void computeGridSize(uint n, uint blockSize, uint &numBlocks,
                     uint &numThreads) {
  numThreads = std::min(blockSize, n);
  numBlocks = iDivUp(n, numThreads);
}


void getLastCudaError(const char *errorMessage, const char *file,
                               const int line) {
  cudaError_t err = cudaGetLastError();

  if (cudaSuccess != err) {
    fprintf(stderr,
            "%s(%i) : getLastCudaError() CUDA error :"
            " %s : (%d) %s.\n",
            file, line, errorMessage, static_cast<int>(err),
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
}

}