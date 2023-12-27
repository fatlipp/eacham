#include "particle.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <stdlib.h>
#include <stdio.h>


__device__ float convolve(const int x, const int y, const int width, const int height,
	unsigned char* inputImage, const char* kernel)
{
	float g = 0;
	for ( int i = 0; i < 3; i++)
	{
		for ( int j = 0; j < 3; j++)
		{
			int dX = x + i;
			int dY = y + j;

			if ( dX < 0 )
				dX = 0;

			if ( dX >= width )
				dX = width - 1;

			if ( dY < 0 )
				dY = 0;

			if ( dY >= height )
				dY = height - 1;

			const int idPixel = dY * width + dX;
			g += inputImage[idPixel] * kernel[j * 3 + i];
		}
	}

	return g;
}

__global__ void SobelKernel(int width, int height, 
		unsigned char* inputImage, unsigned char* outputImage)
{
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

	if (x < width && y < height)
	{
		const char kernel1[9] {-1, 0, 1, -2, 0, 2, -1, 0, 1};
		const char kernel2[9] {-1, -2, -1, 0, 0, 0, 1, 2, 1};
		
		const float g1 = convolve(x, y, width, height, inputImage, kernel1);
		const float g2 = convolve(x, y, width, height, inputImage, kernel2);
		const float g = std::sqrt(g1 * g1 + g2 * g2);

		const int idx = (y * width + x);
		outputImage[idx] = g;
	}
}

int main(int argc, char ** argv)
{
	if (argc < 2)
	{
		return -1;
	}

	cv::Mat inpMat = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
	cv::Mat_<unsigned char> outMat(inpMat.rows, inpMat.cols);

	cv::imwrite("orig.jpg", inpMat);

	std::cout << "size: " << inpMat.cols << " x " << inpMat.rows << std::endl;
	std::cout << "total: " << inpMat.total() << std::endl;
	std::cout << "elemSize: " << inpMat.elemSize() << std::endl;

	auto imageSize = inpMat.total();
	auto totalBytes = inpMat.total() * inpMat.elemSize();

	std::cout << "totalBytes: " << totalBytes << std::endl;

	cudaError_t error;

	error = cudaGetLastError();
	if (error != cudaSuccess)
  	{
		printf("0 %s\n",cudaGetErrorString(error));
		exit(1);
  	}

	unsigned char* inputImageArray = NULL;
	cudaMalloc(&inputImageArray, totalBytes);

	unsigned char* outputImageArray = NULL;
	cudaMalloc(&outputImageArray, totalBytes);

	cudaDeviceSynchronize();
	
	error = cudaGetLastError();
	
	if (error != cudaSuccess)
  	{
        printf("1 %s\n",cudaGetErrorString(error));
        exit(1);
  	}

	cudaMemcpy(inputImageArray, inpMat.data, totalBytes, cudaMemcpyHostToDevice);
	cudaDeviceSynchronize();

	error = cudaGetLastError();
	
	if (error != cudaSuccess)
  	{
        printf("2 %s\n",cudaGetErrorString(error));
        exit(1);
  	}

	dim3 dimBlock(16, 8);
	dim3 dimGrid((inpMat.cols + dimBlock.x - 1) / dimBlock.x, 
				 (inpMat.rows + dimBlock.y - 1) / dimBlock.y);

	SobelKernel<<<dimGrid, dimBlock>>>(inpMat.cols, inpMat.rows, inputImageArray, outputImageArray);
	error = cudaGetLastError();
	if (error != cudaSuccess)
	{
		printf("3 %s\n",cudaGetErrorString(error));
		exit(1);
	}

	cudaDeviceSynchronize();
	cudaMemcpy(outMat.data, outputImageArray, totalBytes, cudaMemcpyDeviceToHost);

	cv::imwrite("changed.jpg", outMat);

	cudaFree(inputImageArray);
	cudaFree(outputImageArray);

	std::cout << "done\n";
	return 0;
}