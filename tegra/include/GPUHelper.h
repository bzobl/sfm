#ifndef GPU_HELPER_H_INCLUDED
#define GPU_HELPER_H_INCLUDED

#include <cuda_runtime.h>
#include <stdexcept>
#include <iostream>
#include <sstream>

#define check(_error)                    check_(_error, __LINE__ , __FILE__ )
#define device_free(_ptr)                device_free_(_ptr, __LINE__ , __FILE__ )

class GPUHelper {

public:
  static inline void check_(cudaError const error, int line, char const *file) {
    if (error != cudaSuccess) {
      std::stringstream sstr;
      sstr << file << ":" << line << " CUDA check failed: ("
           << cudaGetErrorName(error) << "): " << cudaGetErrorString(error);
      throw std::runtime_error(sstr.str());
    }
  }

  template <typename T>
  static T *allocate(size_t count, int line, char const *file)
  {
    T *gpu_mem = nullptr;
    check_(cudaMalloc(&gpu_mem, sizeof(T) * count), line, file);
    return gpu_mem;
  }

  template <typename T>
  static T *allocate(size_t count)
  {
    return allocate<T>(count, __LINE__ , __FILE__);
  }

  static void device_free_(void *ptr, int line, char const *file)
  {
    check_(cudaFree(ptr), line, file);
  }
  
  template <typename T, typename T2>
  static void device_memcpy(T *dst, T2 *src, size_t count, int line, char const *file)
  {
    check_(cudaMemcpy(dst, src, count * sizeof(T), cudaMemcpyHostToDevice), line, file);
  }

  template <typename T, typename T2>
  static void device_memcpy(T *dst, T2 *src, size_t count)
  {
    device_memcpy(dst, src, count, __LINE__ , __FILE__);
  }
};

#endif
