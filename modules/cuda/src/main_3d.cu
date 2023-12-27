#include "particle.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <vector>

__global__ void advanceParticles(unsigned char* inputImage)
{
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    const int z = blockIdx.z * blockDim.z + threadIdx.z;
	const int width = gridDim.x * blockDim.x;
	const int idx = (y * 4 + x) + blockIdx.z * (blockDim.x * blockDim.y * gridDim.x * gridDim.y);

	printf("GPU: xy: (%i, %i, %i), id: %i, data: %i\n", x, y, z, idx, 0);// inputImage[idx]);
}

int main(int argc, char ** argv)
{
	std::vector<unsigned char> data = {};

	for (int i = 0; i < 48; ++i)
		data.push_back(i);

	int totalBytes = data.size() * sizeof(unsigned char);

	unsigned char* inputImageArray = NULL;
	cudaMalloc(&inputImageArray, totalBytes);
	cudaDeviceSynchronize();

	auto error = cudaGetLastError();
	
	if (error != cudaSuccess)
  	{
        printf("1 %s\n",cudaGetErrorString(error));
        exit(1);
  	}

	cudaMemcpy(inputImageArray, data.data(), totalBytes, cudaMemcpyHostToDevice);
	cudaDeviceSynchronize(); 
	error = cudaGetLastError();
	
	if (error != cudaSuccess)
  	{
        printf("2 %s\n",cudaGetErrorString(error));
        exit(1);
  	}

	dim3 dimBlock(2, 2);
	dim3 dimGrid(2, 2, 3);

	advanceParticles<<<dimGrid, dimBlock>>>(inputImageArray);
	error = cudaGetLastError();
	if (error != cudaSuccess)
	{
		printf("3 %s\n",cudaGetErrorString(error));
		exit(1);
	}

	cudaDeviceSynchronize();
	cudaMemcpy(data.data(), inputImageArray, totalBytes, cudaMemcpyDeviceToHost);

	std::cout << "done\n";
	return 0;
}