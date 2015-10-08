#ifndef GPU_ITERATIVE_LINEAR_LS_H_INCLUDED
#define GPU_ITERATIVE_LINEAR_LS_H_INCLUDED

#include "ITriangulation.h"
#include <opencv2/core.hpp>
#include <cublas_v2.h>

#include "cuda/triangulation.cuh"

class GPUIterativeLinearLS : public ITriangulation {
private:

  int mCudaDevice = 1;
  bool mZeroCopy = false;

  cublasHandle_t mCublasHandle;

#ifndef USE_CONSTANT_MEMORY
  float *mGpuP0 = nullptr;
  float *mGpuP1 = nullptr;

  float *mGpuK= nullptr;
  float *mGpuKinv = nullptr;
  float *mGpuKappa = nullptr;
#endif
  bool mCameraPropertiesSet = false;

  size_t mAllocatedKeypoints = 0;
  float **mGpuAarray = nullptr;
  float **mGpuBarray = nullptr;

  Point2D *mGpu_x0 = nullptr;
  Point2D *mGpu_x1 = nullptr;
  Point2D *mGpu_xc0 = nullptr;
  Point2D *mGpu_xc1 = nullptr;

  float *mGpuW0 = nullptr;
  float *mGpuW1 = nullptr;

  int *mGpuDeviceInfo = nullptr;

  Point3D *mGpuPoints3d = nullptr;
  float *mGpuReprErrors = nullptr;

  void setup(size_t pre_allocate_n);
  void selectCudaDevice();

  void freeMemory();

  template <typename T>
  T *allocate(size_t n_elems, bool on_host, int const line, char const * const file);
  void free(void *ptr, bool on_host);
  void allocateLSMatrices(size_t n_keypoints);
  void freeLSMatrices();

  void fillMat(cv::Mat_<float> const &mat, int const size, float *array);
  void copyMat(cv::Mat_<float> const &mat, size_t const size, float *gpu_mat,
               int const line, char const * const file);
  void copyMatConst(cv::Mat_<float> const &mat, size_t const size, float *const_mem_addr,
               int const line, char const * const file);
  void populateConstMatrices(Camera const &cam,
                             cv::Mat_<float> const &P_0, cv::Mat_<float> const &P_1);

  void populateKeypoints(ViewCombination const &view, Point2D *kp_left, Point2D *kp_right);
  void populateKeypoints(ViewCombination const &view);

  template <typename T>
  void mapMemory(T *host_mem, size_t n_elems, T **device_mem);

  bool callKernel(size_t const n_keypoints, int const width, int const height);
  void addToPointCloud(ViewCombination const &view, Point3D const *points, float const *errors,
                       PointCloud &pts);
  bool triangulateCopy(Camera const &cam, ViewCombination const & view, PointCloud &pts);
  bool triangulateZeroCopy(Camera const &cam, ViewCombination const & view, PointCloud &pts);

public:
  GPUIterativeLinearLS(size_t pre_allocate_n = 0, bool map_memory = false);
  virtual ~GPUIterativeLinearLS();

  void allocateMemory(size_t n_keypoints);

  virtual bool triangulate(Camera const &cam, ViewCombination const &view, PointCloud &pts);
};

#endif
