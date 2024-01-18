#pragma once

#include "CudaHelper.h"

#define GET_CUDA_ERROR(msg) cuda::getLastCudaError(msg, __FILE__, __LINE__);