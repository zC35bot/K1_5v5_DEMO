#ifndef TRTX_CUDA_UTILS_H_
#define TRTX_CUDA_UTILS_H_

#include <cuda_runtime_api.h>

#include <iostream>
#include <stdexcept>

#ifndef CUDA_CHECK
#define CUDA_CHECK(callstr)\
    {\
        cudaError_t error_code = callstr;\
        if (error_code != cudaSuccess) {\
            std::cerr << "CUDA error " << cudaGetErrorString(error_code) << " at " << __FILE__ << ":" << __LINE__ << std::endl;\
            throw std::runtime_error(std::string("CUDA error: ") + cudaGetErrorString(error_code));\
        }\
    }
#endif  // CUDA_CHECK

#endif  // TRTX_CUDA_UTILS_H_
