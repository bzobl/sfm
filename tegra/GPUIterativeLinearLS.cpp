#include "GPUIterativeLinearLS.h"

#include <cuda_runtime.h>
#include <iostream>
#include <stdexcept>

#include "GPUHelper.h"
#include "StopWatch.h"

//#define SCOPED_STOP_WATCH(_text) ScopedStopWatch sw(_text)
#define SCOPED_STOP_WATCH(_text) while(0)

GPUIterativeLinearLS::GPUIterativeLinearLS(size_t pre_allocate_n, bool map_memory)
                                          : ITriangulation(), mZeroCopy(map_memory)
{
  setup(pre_allocate_n);
}

GPUIterativeLinearLS::~GPUIterativeLinearLS()
{
  cublasDestroy(mCublasHandle);
  freeMemory();
}

void GPUIterativeLinearLS::setup(size_t pre_allocate_n)
{
  {
    SCOPED_STOP_WATCH("selecting and resetting device");
    selectCudaDevice();
  }

  {
    SCOPED_STOP_WATCH("allocating memory");
    if (pre_allocate_n > 0) {
      allocateMemory(pre_allocate_n);
    }
  }

  {
    SCOPED_STOP_WATCH("creating cublas handle");
    if (cublasCreate_v2(&mCublasHandle) != CUBLAS_STATUS_SUCCESS) {
      std::cerr << "Could not create CUBLAS handle" << std::endl;
    }
  }
}

void GPUIterativeLinearLS::selectCudaDevice()
{
  GPUHelper::check(cudaGetDeviceCount(&mCudaDevice));

  if (mCudaDevice == 0) throw new std::runtime_error("No GPU found");

  mCudaDevice--;

  GPUHelper::check(cudaSetDevice(mCudaDevice));
  GPUHelper::check(cudaDeviceReset());
  if (mZeroCopy) {
    struct cudaDeviceProp prop;
    GPUHelper::check(cudaGetDeviceProperties(&prop, mCudaDevice));
    if (!prop.canMapHostMemory) throw new std::runtime_error("GPU cannot map host memory");

    GPUHelper::check(cudaSetDeviceFlags(cudaDeviceMapHost));
  }
}

template <typename T>
T *GPUIterativeLinearLS::allocate(size_t n_elems, bool on_host,
                                  int const line, char const * const file)
{
  T *ptr = nullptr;

  if (on_host) {
    GPUHelper::check(cudaHostAlloc(&ptr, n_elems * sizeof(T), cudaHostAllocMapped));
  } else {
    ptr = GPUHelper::allocate<T>(n_elems, line, file);
  }
  return ptr;
}

void GPUIterativeLinearLS::free(void *ptr, bool on_host)
{
  if (on_host) {
    GPUHelper::check(cudaFreeHost(ptr));
  } else {
    GPUHelper::device_free(ptr);
  }
}

void GPUIterativeLinearLS::allocateMemory(size_t n_keypoints)
{
#ifndef USE_CONSTANT_MEMORY
  // Projection Matrices
  if (mGpuP0 == nullptr) {
    assert(mGpuP1 == nullptr);
    mGpuP0 = allocate<float>(P_SIZE, false, __LINE__ , __FILE__ );
    mGpuP1 = allocate<float>(P_SIZE, false, __LINE__ , __FILE__ );
  }

  // Camera Matrices
  if (mGpuK == nullptr) {
    assert(mGpuKinv == nullptr);
    mGpuK = allocate<float>(K_SIZE, false, __LINE__ , __FILE__ );
    mGpuKinv = allocate<float>(K_SIZE, false, __LINE__ , __FILE__ );
  }

  // Distortion coefficients
  if (mGpuKappa == nullptr) {
    mGpuKappa = allocate<float>(KAPPA_SIZE, false, __LINE__ , __FILE__ );
  }
#endif

  if (n_keypoints > mAllocatedKeypoints) {
    // image coordinates
    if (mGpu_x0 != nullptr) {
      assert(mGpu_x1 != nullptr);
      this->free(mGpu_x0, mZeroCopy);
      this->free(mGpu_x1, mZeroCopy);
    }
    mGpu_x0 = allocate<Point2D>(n_keypoints, mZeroCopy, __LINE__ , __FILE__ );
    mGpu_x1 = allocate<Point2D>(n_keypoints, mZeroCopy, __LINE__ , __FILE__ );

    // normalized coordinates
    if (mGpu_xc0 != nullptr) {
      assert(mGpu_xc1 != nullptr);
      this->free(mGpu_xc0, false);
      this->free(mGpu_xc1, false);
    }
    mGpu_xc0 = allocate<Point2D>(n_keypoints, false, __LINE__ , __FILE__ );
    mGpu_xc1 = allocate<Point2D>(n_keypoints, false, __LINE__ , __FILE__ );

    // weights
    if (mGpuW0 != nullptr) {
      assert(mGpuW1 != nullptr);
      this->free(mGpuW0, false);
      this->free(mGpuW1, false);
    }
    mGpuW0 = allocate<float>(n_keypoints, false, __LINE__ , __FILE__ );
    mGpuW1 = allocate<float>(n_keypoints, false, __LINE__ , __FILE__ );

    // device info
    if (mGpuDeviceInfo != nullptr) {
      this->free(mGpuDeviceInfo, false);
    }
    mGpuDeviceInfo = allocate<int>(n_keypoints, false, __LINE__ , __FILE__ );

    // 3D Points
    if (mGpuPoints3d != nullptr) {
      this->free(mGpuPoints3d, mZeroCopy);
    }
    mGpuPoints3d = allocate<Point3D>(n_keypoints, mZeroCopy, __LINE__ , __FILE__ );

    // reprojection errors
    if (mGpuReprErrors != nullptr) {
      this->free(mGpuReprErrors, mZeroCopy);
    }
    mGpuReprErrors = allocate<float>(n_keypoints, mZeroCopy, __LINE__ , __FILE__ );

    allocateLSMatrices(n_keypoints);
    mAllocatedKeypoints = n_keypoints;
  }
}

void GPUIterativeLinearLS::freeMemory()
{
  freeLSMatrices();

  if (mZeroCopy) return;
 
#ifndef USE_CONSTANT_MEMORY
  this->free(mGpuP0, false);
  this->free(mGpuP1, false);

  this->free(mGpuK, false);
  this->free(mGpuKinv, false);
  this->free(mGpuKappa, false);
#endif

  this->free(mGpu_x0, mZeroCopy);
  this->free(mGpu_x1, mZeroCopy);
  this->free(mGpu_xc0, false);
  this->free(mGpu_xc1, false);

  this->free(mGpuW0, false);
  this->free(mGpuW1, false);

  this->free(mGpuDeviceInfo, false);

  this->free(mGpuPoints3d, mZeroCopy);
  this->free(mGpuReprErrors, mZeroCopy);
}

void GPUIterativeLinearLS::allocateLSMatrices(size_t const n_keypoints)
{
  freeLSMatrices();

  float *temp_ptr_A[n_keypoints];
  float *temp_ptr_B[n_keypoints];

  // allocate array of matrices
  mGpuAarray = allocate<float *>(n_keypoints, false, __LINE__ , __FILE__ );
  mGpuBarray = allocate<float *>(n_keypoints, false, __LINE__ , __FILE__ );

  // allocate n_keypoints matrices
  temp_ptr_A[0] = allocate<float>(n_keypoints * 12, false, __LINE__ , __FILE__ );
  temp_ptr_B[0] = allocate<float>(n_keypoints * 4, false, __LINE__ , __FILE__ );

  // populate pointer array with pointer to the individual matrices
  for (size_t i = 1; i < n_keypoints; i++) {
    temp_ptr_A[i] = temp_ptr_A[i - 1] + 12;
    temp_ptr_B[i] = temp_ptr_B[i - 1] + 4;
  }

  // copy pointer array to GPU
  GPUHelper::device_memcpy(mGpuAarray, temp_ptr_A, n_keypoints, __LINE__ , __FILE__);
  GPUHelper::device_memcpy(mGpuBarray, temp_ptr_B, n_keypoints, __LINE__ , __FILE__);
}

void GPUIterativeLinearLS::freeLSMatrices()
{
  if (mGpuAarray != nullptr) {
    float *temp_ptr_A[1];
    GPUHelper::check(cudaMemcpy(temp_ptr_A, mGpuAarray, sizeof(float *), cudaMemcpyDeviceToHost));
    this->free(temp_ptr_A[0], false);
    this->free(mGpuAarray, false);
  }

  if (mGpuBarray != nullptr) {
    float *temp_ptr_B[1];
    GPUHelper::check(cudaMemcpy(temp_ptr_B, mGpuBarray, sizeof(float *), cudaMemcpyDeviceToHost));
    this->free(temp_ptr_B[0], false);
    this->free(mGpuBarray, false);
  }
}

void GPUIterativeLinearLS::populateKeypoints(ViewCombination const &view,
                                             Point2D *kp_left, Point2D *kp_right)
{
  for (size_t i = 0; i < view.getMatchesSize(); i++) {
    kp_left[i].x = view.getMatchingPointLeft(i).x;
    kp_left[i].y = view.getMatchingPointLeft(i).y;
    kp_left[i].w = 1;

    kp_right[i].x = view.getMatchingPointRight(i).x;
    kp_right[i].y = view.getMatchingPointRight(i).y;
    kp_right[i].w = 1;
  }
}

void GPUIterativeLinearLS::fillMat(cv::Mat_<float> const &mat, int const size, float *array)
{
  assert((mat.rows * mat.cols) == size);
  int pos = 0;
  for (int r = 0; r < mat.rows; r++) {
    for (int c = 0; c < mat.cols; c++) {
      array[pos++] = mat(r, c);
    }
  }
}

void GPUIterativeLinearLS::copyMat(cv::Mat_<float> const &mat, size_t const size,
                                   float *gpu_mat, int const line, char const * const file)
{
  if (mat.isContinuous()) {
    GPUHelper::device_memcpy<float>(gpu_mat, mat.ptr(), size, line, file);
  } else {
    float m[size];
    fillMat(mat, size, m);
    GPUHelper::device_memcpy<float>(gpu_mat, m, size, line, file);
  }
}

void GPUIterativeLinearLS::copyMatConst(cv::Mat_<float> const &mat, size_t const size,
                                        float *const_mem_addr, int const line, char const * const file)
{
  if (mat.isContinuous()) {
    GPUHelper::check(cudaMemcpyToSymbol(const_mem_addr, mat.ptr(),
                                        size * sizeof(float), 0, cudaMemcpyHostToDevice));
  } else {
    float m[size];
    fillMat(mat, size, m);
    GPUHelper::check(cudaMemcpyToSymbol(const_mem_addr, m, size * sizeof(float), 0, cudaMemcpyHostToDevice));
  }
}

void GPUIterativeLinearLS::populateConstMatrices(Camera const &cam,
                                                 cv::Mat_<float> const &P_0,
                                                 cv::Mat_<float> const &P_1)
{
  cv::Mat_<float> K = cam.getCalibrationMatrix();

#ifndef USE_CONSTANT_MEMORY
  // supply P_0 and P_1 as arguments to kernel
  copyMat(P_0, P_SIZE, mGpuP0, __LINE__ , __FILE__);
  copyMat(P_1, P_SIZE, mGpuP1, __LINE__ , __FILE__);
#else
  // use constant memory to save projection matrices
  copyMatConst(P_0, P_SIZE, cuP_0, __LINE__ , __FILE__);
  copyMatConst(P_1, P_SIZE, cuP_1, __LINE__ , __FILE__);
#endif

  if (!mCameraPropertiesSet) {
    cv::Mat_<float> K_inv;
    cv::Mat_<float> kappa = cam.getDistortionCoefficients();
    cv::invert(K, K_inv);

#ifndef USE_CONSTANT_MEMORY
    // supply K, K_inv and kappa as arguments to kernel
    copyMat(K, K_SIZE, mGpuK, __LINE__ , __FILE__);
    copyMat(K_inv, K_SIZE, mGpuKinv, __LINE__ , __FILE__);
    copyMat(kappa, KAPPA_SIZE, mGpuKappa, __LINE__ , __FILE__);
#else
    // use constant memory to save camera properties permanently
    copyMatConst(K_inv, K_SIZE, cuK_inv, __LINE__ , __FILE__);
    copyMatConst(kappa, KAPPA_SIZE, cuKappa, __LINE__ , __FILE__);
#endif
    mCameraPropertiesSet = true;
  }

#ifdef USE_CONSTANT_MEMORY
  // calculate KP0 and KP1
  cv::Mat_<float> KP0 = K * P_0;
  cv::Mat_<float> KP1 = K * P_1;
  copyMatConst(KP0, P_SIZE, cuKP0, __LINE__ , __FILE__);
  copyMatConst(KP1, P_SIZE, cuKP1, __LINE__ , __FILE__);
#endif
}

bool iteration_callback(float **A_array, float **B_array, int n_points, int *dev_info,
                        void *user_data)
{
  cublasHandle_t *handle = (cublasHandle_t *)user_data;

  int info;
  if (cublasSgelsBatched(*handle, CUBLAS_OP_N,
                         4, 3, 1, A_array, 4, B_array, 4,
                         &info, dev_info, n_points) != CUBLAS_STATUS_SUCCESS) {
    std::cerr << "Least Squares Solver: info=" << info << std::endl;
    return false;
  }
  return true;
}

bool GPUIterativeLinearLS::callKernel(size_t const n_keypoints, int const width, int const height)
{
  struct cudaDeviceProp prop;
  dim3 num_blocks;
  dim3 threads_per_block;
  {
    SCOPED_STOP_WATCH("calculating sizes");
    GPUHelper::check(cudaGetDeviceProperties(&prop, mCudaDevice));

    // for TEGRA, maximum threads are limited by the registers available (32K)
    // only 799 threads are possible with current triangulation implementation
    int const x_threads = prop.maxThreadsPerBlock;
    int const x_blocks = (n_keypoints + x_threads - 1) / x_threads;
    num_blocks = dim3(x_blocks, 1, 1);
    threads_per_block = dim3(x_threads, 1, 1);

    //std::cout << "x_threads: " << x_threads << ", x_blocks: " << x_blocks << std::endl;

    if (x_blocks > prop.maxGridSize[0]) {
      std::cerr << "Blocks exceed maximum grid size" << std::endl;
      return false;
    }
  }

  {
    SCOPED_STOP_WATCH("calling kernel");
    IterativeLSArguments args;
    args.n_keypoints = n_keypoints;
#ifndef USE_CONSTANT_MEMORY
    args.P_0 = mGpuP0;
    args.P_1 = mGpuP1;
    args.K = mGpuK;
    args.K_inv = mGpuKinv;
    args.kappa = mGpuKappa;
#endif
    args.width = width;
    args.height = height;
    args.kp_left = mGpu_x0;
    args.kp_right = mGpu_x1;
    args.xc_left = mGpu_xc0;
    args.xc_right = mGpu_xc1;
    args.A_array = mGpuAarray;
    args.B_array = mGpuBarray;
    args.w_0 = mGpuW0;
    args.w_1 = mGpuW1;
    args.device_info = mGpuDeviceInfo;
    args.points_3d = mGpuPoints3d;
    args.repr_errors = mGpuReprErrors;

    iterative_linear_ls_wrapper(num_blocks, threads_per_block,
                                args, iteration_callback, &mCublasHandle);
    GPUHelper::check(cudaDeviceSynchronize());
  }

  return true;
}

void GPUIterativeLinearLS::addToPointCloud(ViewCombination const &view,
                                           Point3D const *points, float const *errors,
                                           PointCloud &pts)
{
  for (size_t i = 0; i < view.getMatchesSize(); i++) {
    pts.addPoint(cv::Vec4d(points[i].x, points[i].y, points[i].z, points[i].w), errors[i],
                 std::make_pair(view.getImageLeft(), view.getMatch(i).queryIdx),
                 std::make_pair(view.getImageRight(), view.getMatch(i).trainIdx));
  }
}

bool GPUIterativeLinearLS::triangulateCopy(Camera const &cam, ViewCombination const &view,
                                           PointCloud &pts)
{
  size_t n_keypoints = view.getMatchesSize();
  Point2D kp_left[n_keypoints];
  Point2D kp_right[n_keypoints];
  Point3D points_3d[n_keypoints];
  float errors[n_keypoints];

  {
    SCOPED_STOP_WATCH("copying data");
    populateKeypoints(view, kp_left, kp_right);
    GPUHelper::device_memcpy<Point2D>(mGpu_x0, &kp_left[0], n_keypoints, __LINE__ , __FILE__);
    GPUHelper::device_memcpy<Point2D>(mGpu_x1, &kp_right[0], n_keypoints, __LINE__ , __FILE__);

    populateConstMatrices(cam, 
                          view.getImageLeft()->getProjectionMatrix(),
                          view.getImageRight()->getProjectionMatrix());
  }

  callKernel(n_keypoints, cam.getWidth(), cam.getHeight());

  {
    SCOPED_STOP_WATCH("copying back data");
    GPUHelper::check(cudaMemcpy(points_3d, mGpuPoints3d, sizeof(Point3D) * n_keypoints, cudaMemcpyDeviceToHost));
    GPUHelper::check(cudaMemcpy(errors, mGpuReprErrors, sizeof(float) * n_keypoints, cudaMemcpyDeviceToHost));
    addToPointCloud(view, points_3d, errors, pts);
  }
  return true;
}

template <typename T>
void GPUIterativeLinearLS::mapMemory(T *host_mem, size_t n_elems, T **device_mem)
{
  GPUHelper::check(cudaHostRegister(host_mem, n_elems * sizeof(T), cudaHostRegisterMapped | cudaHostRegisterPortable));
  GPUHelper::check(cudaHostGetDevicePointer(device_mem, host_mem, 0));
}

bool GPUIterativeLinearLS::triangulateZeroCopy(Camera const &cam, ViewCombination const &view,
                                               PointCloud &pts)
{
  size_t n_keypoints = view.getMatchesSize();
  {
    SCOPED_STOP_WATCH("copying data");
    // when using zero copy, the gpu arrays can be filled directly
    populateKeypoints(view, mGpu_x0, mGpu_x1);
    populateConstMatrices(cam,
                          view.getImageLeft()->getProjectionMatrix(),
                          view.getImageRight()->getProjectionMatrix());
  }

  callKernel(n_keypoints, cam.getWidth(), cam.getHeight());

  {
    SCOPED_STOP_WATCH("copying back data");
    // when using zero copy, the gpu arrays can be accessed directly
    addToPointCloud(view, mGpuPoints3d, mGpuReprErrors, pts);
  }

  return true;
}

bool GPUIterativeLinearLS::triangulate(Camera const & cam, ViewCombination const &view,
                                       PointCloud &pts)
{
  bool success;

  {
    SCOPED_STOP_WATCH("allocating memory");
    allocateMemory(view.getMatchesSize());
  }

  if (!mZeroCopy) {
    success = triangulateCopy(cam, view, pts);
  } else {
    success = triangulateZeroCopy(cam, view, pts);
  }
  return success;
}
