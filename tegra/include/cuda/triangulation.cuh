#ifndef TRIANGULATION_CUH
#define TRIANGULATION_CUH

#include <cuda_runtime.h>

struct Point2D {
  float x;
  float y;
  float w;
};

struct Point3D {
  float x;
  float y;
  float z;
  float w;
};

#define P_SIZE (3 * 4)
#define K_SIZE (3 * 3)
#define KAPPA_SIZE (1 * 5)
typedef float *ProjectionMatrix;
typedef float *CalibrationMatrix;
typedef float *DistortionCoefficients;

typedef bool (*iteration_cb)(float **A_array, float **B_array, int n_keypoints, int *dev_info,
                             void *user_data);

struct IterativeLSArguments {
  std::size_t n_keypoints;
#ifndef USE_CONSTANT_MEMORY
  ProjectionMatrix P_0;
  ProjectionMatrix P_1;
  CalibrationMatrix K;
  CalibrationMatrix K_inv;
  DistortionCoefficients kappa;
#endif
  std::size_t width;
  std::size_t height;
  Point2D *kp_left;
  Point2D *kp_right;
  Point2D *xc_left;
  Point2D *xc_right;
  float **A_array;
  float **B_array;
  float *w_0;
  float *w_1;
  int *device_info;
  Point3D *points_3d;
  float *repr_errors;
};

#ifdef USE_CONSTANT_MEMORY
extern float cuP_0[];
extern float cuP_1[];

extern float cuKP0[];
extern float cuKP1[];
extern float cuK_inv[];
extern float cuKappa[];
#endif

void iterative_linear_ls_wrapper(dim3 const &b_p_g, dim3 const &t_p_b,
                                 IterativeLSArguments args,
                                 iteration_cb doLS, void *user_data);
#endif
