#ifndef INFERTOOL_H
#define INFERTOOL_H

#include <cuda_runtime.h>
#include <cstdint>
#include <BaseInfer.hpp>

namespace tdt_radar {

#define checkRuntime(call)                                                 \
    do {                                                                   \
        auto ___call__ret_code__ = (call);                                 \
        if (___call__ret_code__ != cudaSuccess) {                          \
            INFO("CUDA Runtime error💥 %s # %s, code = %s [ %d ]", #call,  \
                 cudaGetErrorString(___call__ret_code__),                  \
                 cudaGetErrorName(___call__ret_code__),                    \
                 ___call__ret_code__);                                     \
            abort();                                                       \
        }                                                                  \
    } while (0)

#define checkKernel(...)                                                   \
    do {                                                                   \
        {                                                                  \
            (__VA_ARGS__);                                                 \
        }                                                                  \
        checkRuntime(cudaPeekAtLastError());                               \
    } while (0)

__global__ void warp_affine_bilinear_and_normalize_plane_kernel(
    uint8_t* src, int src_line_size, int src_width, int src_height,
    float* dst, int dst_width, int dst_height, uint8_t const_value_st,
    float* warp_affine_matrix_2_3, Norm norm);

void warp_affine_bilinear_and_normalize_plane(
    uint8_t* src, int src_line_size, int src_width, int src_height,
    float* dst, int dst_width, int dst_height, float* matrix_2_3,
    uint8_t const_value, const tdt_radar::Norm& norm, cudaStream_t stream);

}  // namespace tdt_radar

#endif
