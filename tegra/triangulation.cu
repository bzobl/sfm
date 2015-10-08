#include "cuda/triangulation.cuh"

#include <stdio.h>
#include <iostream>

#include "GPUHelper.h"

/* Convenience defines for the access of projection matrices
 *
 *  p11 = P_0[0]; p12 = P_0[1]; p13 = P_0[2];  p14 = P_0[3];
 *  p21 = P_0[4]; p22 = P_0[5]; p23 = P_0[6];  p24 = P_0[7];
 *  p31 = P_0[8]; p32 = P_0[9]; p33 = P_0[10]; p34 = P_0[11];
 *
 *  q11 = P_1[0]; q12 = P_1[1]; q13 = P_1[2];  q14 = P_1[3];
 *  q21 = P_1[4]; q22 = P_1[5]; q23 = P_1[6];  q24 = P_1[7];
 *  q31 = P_1[8]; q32 = P_1[9]; q33 = P_1[10]; q34 = P_1[11];
 *
 *  u0 = x_0.x;
 *  v0 = x_0.y;
 *  u1 = x_1.x;
 *  v1 = x_1.y;
 */

#define p11 P_0[0]
#define p12 P_0[1]
#define p13 P_0[2]
#define p14 P_0[3]
#define p21 P_0[4]
#define p22 P_0[5]
#define p23 P_0[6]
#define p24 P_0[7]
#define p31 P_0[8]
#define p32 P_0[9]
#define p33 P_0[10]
#define p34 P_0[11]

#define q11 P_1[0]
#define q12 P_1[1]
#define q13 P_1[2]
#define q14 P_1[3]
#define q21 P_1[4]
#define q22 P_1[5]
#define q23 P_1[6]
#define q24 P_1[7]
#define q31 P_1[8]
#define q32 P_1[9]
#define q33 P_1[10]
#define q34 P_1[11]

#define u0 x_0.x
#define v0 x_0.y
#define u1 x_1.x
#define v1 x_1.y

#ifdef USE_CONSTANT_MEMORY
__constant__ float cuP_0[P_SIZE];
__constant__ float cuP_1[P_SIZE];

__constant__ float cuKP0[P_SIZE];
__constant__ float cuKP1[P_SIZE];
__constant__ float cuK_inv[K_SIZE];
__constant__ float cuKappa[KAPPA_SIZE];
#endif

#ifdef PRINT
__device__
inline void printMatrices(IterativeLSArguments const &args,
                          ProjectionMatrix const KP0,
                          ProjectionMatrix const KP1)
{
#ifndef USE_CONSTANT_MEMORY
  printf("P_0 = [%.02f, %.02f, %.02f, %.02f]\n"
         "      [%.02f, %.02f, %.02f, %.02f]\n"
         "      [%.02f, %.02f, %.02f, %.02f]\n",
         args.P_0[0], args.P_0[1], args.P_0[2], args.P_0[3],
         args.P_0[4], args.P_0[5], args.P_0[6], args.P_0[7],
         args.P_0[8], args.P_0[9], args.P_0[10],args.P_0[11]);
  printf("P_1 = [%.02f, %.02f, %.02f, %.02f]\n"
         "      [%.02f, %.02f, %.02f, %.02f]\n"
         "      [%.02f, %.02f, %.02f, %.02f]\n",
         args.P_1[0], args.P_1[1], args.P_1[2], args.P_1[3],
         args.P_1[4], args.P_1[5], args.P_1[6], args.P_1[7],
         args.P_1[8], args.P_1[9], args.P_1[10],args.P_1[11]);
  printf("kappa = [%.02f, %.02f, %.02f, %.02f, %.02f ]\n",
         args.kappa[0], args.kappa[1], args.kappa[2], args.kappa[3], args.kappa[4]);
  printf("K = [%.02f, %.02f, %.02f, ]\n"
         "    [%.02f, %.02f, %.02f ]\n"
         "    [%.02f, %.02f, %.02f ]\n",
         args.K[0], args.K[1], args.K[2],
         args.K[3], args.K[4], args.K[5],
         args.K[6], args.K[7], args.K[8]);
#else
  printf("P_0 = [%.02f, %.02f, %.02f, %.02f]\n"
         "      [%.02f, %.02f, %.02f, %.02f]\n"
         "      [%.02f, %.02f, %.02f, %.02f]\n",
         cuP_0[0], cuP_0[1], cuP_0[2], cuP_0[3],
         cuP_0[4], cuP_0[5], cuP_0[6], cuP_0[7],
         cuP_0[8], cuP_0[9], cuP_0[10],cuP_0[11]);
  printf("P_1 = [%.02f, %.02f, %.02f, %.02f]\n"
         "      [%.02f, %.02f, %.02f, %.02f]\n"
         "      [%.02f, %.02f, %.02f, %.02f]\n",
         cuP_1[0], cuP_1[1], cuP_1[2], cuP_1[3],
         cuP_1[4], cuP_1[5], cuP_1[6], cuP_1[7],
         cuP_1[8], cuP_1[9], cuP_1[10],cuP_1[11]);
  printf("kappa = [%.02f, %.02f, %.02f, %.02f, %.02f ]\n",
         cuKappa[0], cuKappa[1], cuKappa[2], cuKappa[3], cuKappa[4]);
  printf("K_inv = [%.02f, %.02f, %.02f, ]\n"
         "        [%.02f, %.02f, %.02f ]\n"
         "        [%.02f, %.02f, %.02f ]\n",
         cuK_inv[0], cuK_inv[1], cuK_inv[2],
         cuK_inv[3], cuK_inv[4], cuK_inv[5],
         cuK_inv[6], cuK_inv[7], cuK_inv[8]);
#endif
  printf("KP0 = [%.02f, %.02f, %.02f, %.02f]\n"
         "      [%.02f, %.02f, %.02f, %.02f]\n"
         "      [%.02f, %.02f, %.02f, %.02f]\n",
         KP0[0], KP0[1], KP0[2], KP0[3],
         KP0[4], KP0[5], KP0[6], KP0[7],
         KP0[8], KP0[9], KP0[10], KP0[11]);
  printf("KP1 = [%.02f, %.02f, %.02f, %.02f]\n"
         "      [%.02f, %.02f, %.02f, %.02f]\n"
         "      [%.02f, %.02f, %.02f, %.02f]\n",
         KP1[0], KP1[1], KP1[2], KP1[3],
         KP1[4], KP1[5], KP1[6], KP1[7],
         KP1[8], KP1[9], KP1[10], KP1[11]);
}
#endif

__device__
void normalize_image_point(Point2D const &x, CalibrationMatrix const Kinv, Point2D &xc)
{
  xc.x = Kinv[0] * x.x + Kinv[1] * x.y + Kinv[2] * x.w;
  xc.y = Kinv[3] * x.x + Kinv[4] * x.y + Kinv[5] * x.w;
  xc.w = Kinv[6] * x.x + Kinv[7] * x.y + Kinv[8] * x.w;

#ifdef PRINT
  printf("GPU normalized x = [ %.02f, %.02f, %.02f ] --> xc = [ %.02f, %.02f, %.02f ]\n",
          x.x, x.y, x.w, xc.x, xc.y, xc.w);
#endif
}

__device__
float recalc_weight(float *const &X, ProjectionMatrix const P)
{
  float w = X[0] * P[8] + X[1] * P[9] + X[2] * P[10] + P[11];
#ifdef PRINT
  printf("X = [%.02f, %.02f, %.02f], new weight = %d\n", X[0], X[1], X[2], w);
#endif
  return w;
}

__global__
void recalc_weights(int iteration, IterativeLSArguments args)
{
  int const i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= args.n_keypoints) {
    return;
  }

  if (args.device_info[i] != 0) {
    printf("Thread %d did not solve ls in iteration %d\n", i, iteration);
  }

  float const CONVERGENCE_THRESHOLD = 1e-9;

#ifndef USE_CONSTANT_MEMORY
  float *cuP_0 = args.P_0;
  float *cuP_1 = args.P_1;
#endif

  if (args.repr_errors[i] < 0.0) {
    float temp_w_0 = recalc_weight(args.B_array[i], cuP_0);
    float temp_w_1 = recalc_weight(args.B_array[i], cuP_1);

    if ((std::fabs(args.w_0[i] - temp_w_0) < CONVERGENCE_THRESHOLD) &&
        (std::fabs(args.w_0[i] - temp_w_0) < CONVERGENCE_THRESHOLD)) {

      args.repr_errors[i] = 0.0;
      args.points_3d[i].x = args.B_array[i][0];
      args.points_3d[i].y = args.B_array[i][1];
      args.points_3d[i].z = args.B_array[i][2];
      args.points_3d[i].w = 1;
    }

    args.w_0[i] = temp_w_0;
    args.w_1[i] = temp_w_1;

#ifdef PRINT
    printf("GPU iteration %d: xc = [ %.02f, %.02f, %.02f ] --> X = [ %.02f, %.02f, %.02f ]\n",
           iteration, args.xc_left[i].x, args.xc_left[i].y, args.xc_left[i].w,
           args.B_array[i][0], args.B_array[i][1], args.B_array[i][2]);
#endif
  }
}

__device__
void apply_distortion(Point2D &pt, DistortionCoefficients const kappa,
                      int const width, int const height)
{
  // convert point to normalized image coordinates
  float x = (pt.x * 2 - width) / width;
  float y = (pt.y * 2 - height) / height;
  float r2 = (x * x + y * y) * (x * x + y * y);

  // radial distortion
  float radial = (1 + kappa[0] * r2
                    + kappa[1] * r2*r2
                    + kappa[2] * r2*r2*r2);

  // tangential distortion
  float d_x = 2 * kappa[3] * x * y + kappa[4] * (r2 + 2 * x*x);
  float d_y = kappa[3] * (r2 + 2 * y*y) + 2 * kappa[4] * x * y;

  // normalized distorted image coordinates
  float u = x * radial + d_x;
  float v = y * radial + d_y;

  pt.x = (u + 1) * width / 2;
  pt.y = (v + 1) * height / 2;
}

__device__
inline void fillKP(float *KP, ProjectionMatrix const P, CalibrationMatrix const K, int kp_idx)
{
  int row = kp_idx / 4;
  int col = kp_idx % 4;

  KP[kp_idx] = 0;
  for (int i = 0; i < 3; i++) {
    KP[kp_idx] += K[row * 3 + i] * P[i * 4 + col];
  }
}

__device__
inline void populateKP(CalibrationMatrix const K, ProjectionMatrix const P, size_t n_keypoints,
                       float *KP)
{
  int const thread_idx = blockIdx.x * blockDim.x + threadIdx.x;
  // if less than 12 keypoints are processed, KP is calculated by first thread only
  // if more than 12 keypoints are processd, KP is calculated by first 12 threads
  if (n_keypoints < P_SIZE) {
    if (thread_idx == 0) {
      for (int n = 0; n < P_SIZE; n++) {
        fillKP(KP, P, K, n);
      }
    }
  } else if (thread_idx < P_SIZE) {
    fillKP(KP, P, K, thread_idx);
  }
}

__device__
float reprojection_error(ProjectionMatrix const KP, DistortionCoefficients const kappa, 
                         int const width, int const height, Point2D const &kp, Point3D const &X)
{
  Point2D p_r;

  // x = K * P * X
  p_r.x = X.x * KP[0] + X.y * KP[1] + X.z * KP[2] + X.w * KP[3];
  p_r.y = X.x * KP[4] + X.y * KP[5] + X.z * KP[6] + X.w * KP[7];
  p_r.w = X.x * KP[8] + X.y * KP[9] + X.z * KP[10] + X.w * KP[11];

  // convert to Euclidean
  p_r.x /= p_r.w;
  p_r.y /= p_r.w;

#ifdef PRINT
  printf("GPU distorting x = [ %.02f, %.02f ]\n", p_r.x, p_r.y);
#endif

  apply_distortion(p_r, kappa, width, height);

#ifdef PRINT
  printf(" --> xd = [ %.02f, %.02f ]\n", p_r.x, p_r.y);
#endif

  // difference to original point
  p_r.x -= kp.x;
  p_r.y -= kp.y;

  float error = sqrt(p_r.x * p_r.x + p_r.y * p_r.y);

#ifdef PRINT
  printf(" --> xd - kp = [ %.02f, %.02f ]\n", p_r.x, p_r.y);
  printf(" --> error = %.02f\n", error);
#endif

  // L2 norm
  return error;
}


__device__
inline
void buildA(ProjectionMatrix const P_0, Point2D const &x_0, float const w_0,
            ProjectionMatrix const P_1, Point2D const &x_1, float const w_1,
            float *A)
{
  float wi0 = 1 / w_0;
  float wi1 = 1 / w_1;

  // be aware of column major format!
  // indices:
  // 0 4  8 
  // 1 5  9
  // 2 6 10
  // 3 7 11
  A[0] = wi0 * (p11 - (u0 * p31));
  A[4] = wi0 * (p12 - (u0 * p32));
  A[8] = wi0 * (p13 - (u0 * p33));
  A[1] = wi0 * (p21 - (v0 * p31));
  A[5] = wi0 * (p22 - (v0 * p32));
  A[9] = wi0 * (p23 - (v0 * p33));

  A[2]  = wi1 * (q11 - (u1 * q31));
  A[6]  = wi1 * (q12 - (u1 * q32));
  A[10] = wi1 * (q13 - (u1 * q33));
  A[3]  = wi1 * (q21 - (v1 * q31));
  A[7]  = wi1 * (q22 - (v1 * q32));
  A[11] = wi1 * (q23 - (v1 * q33));
}

__device__
inline
void buildB(ProjectionMatrix const P_0, Point2D const &x_0, float const w_0,
            ProjectionMatrix const P_1, Point2D const &x_1, float const w_1,
            float *B)
{
  float wi0 = 1 / w_0;
  float wi1 = 1 / w_1;

  B[0] = wi0 * ((u0 * p34) - p14);
  B[1] = wi0 * ((v0 * p34) - p24);
  B[2] = wi1 * ((u1 * q34) - q14);
  B[3] = wi1 * ((v1 * q34) - q24);
}

__global__
void prepare_triangulation(IterativeLSArguments args)
{
  int const i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= args.n_keypoints) {
    return;
  }

#ifndef USE_CONSTANT_MEMORY
  float *cuK_inv = args.K_inv;
#endif

  normalize_image_point(args.kp_left[i], cuK_inv, args.xc_left[i]);
  normalize_image_point(args.kp_right[i], cuK_inv, args.xc_right[i]);

  args.w_0[i] = 1;
  args.w_1[i] = 1;
  args.device_info[i] = 0;
  // a negative reprojection error indicates that the triangulation is not yet finished
  args.repr_errors[i] = -1.0;
}

__global__
void cleanup_triangulation(IterativeLSArguments args)
{
  int const i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= args.n_keypoints) {
    return;
  }

  if (args.repr_errors[i] < 0.0) {
#ifdef PRINT
    printf("cleanup triangulation --> X = [ %.02f, %.02f, %.02f ]\n",
           args.B_array[i][0], args.B_array[i][1], args.B_array[i][2]);
#endif
    args.repr_errors[i] = 0.0;
    args.points_3d[i].x = args.B_array[i][0];
    args.points_3d[i].y = args.B_array[i][1];
    args.points_3d[i].z = args.B_array[i][2];
    args.points_3d[i].w = 1;
  }

#ifndef USE_CONSTANT_MEMORY
  __shared__ float cuKP0[P_SIZE];
  __shared__ float cuKP1[P_SIZE];

  populateKP(args.K, args.P_0, args.n_keypoints, cuKP0);
  populateKP(args.K, args.P_1, args.n_keypoints, cuKP1);
  __syncthreads();

  float *cuKappa = args.kappa;
#endif

#ifdef PRINT
  if (i == 0) {
    printMatrices(args, cuKP0, cuKP1);
  }
#endif

  // all threads have to calculate the repr. error, since KP has to be processed
  //float error_left = reprojection_error(cuKP0, cuKappa, args.width, args.height,
  //                                      args.kp_left[i], args.points_3d[i]);
  float error_right = reprojection_error(cuKP1, cuKappa, args.width, args.height,
                                         args.kp_right[i], args.points_3d[i]);
  args.repr_errors[i] = error_right;
  //args.repr_errors[i] = (error_left > error_right) ? error_left : error_right;

  /*
  if (!is_success) {
    args.points_3d[i].x = 0;
    args.points_3d[i].y = 0;
    args.points_3d[i].z = 0;
    args.points_3d[i].w = 0;
    args.repr_errors[i] = 0;
  }
  */
}

__global__
void prepare_iteration(int iteration, IterativeLSArguments args)
{
  int const i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= args.n_keypoints) {
    return;
  }

  if (args.repr_errors[i] >= 0.0) {
    return;
  }

#ifndef USE_CONSTANT_MEMORY
  float *cuP_0 = args.P_0;
  float *cuP_1 = args.P_1;
#endif

  buildA(cuP_0, args.xc_left[i],  args.w_0[i],
         cuP_1, args.xc_right[i], args.w_1[i], args.A_array[i]);
  buildB(cuP_0, args.xc_left[i],  args.w_0[i],
         cuP_1, args.xc_right[i], args.w_1[i], args.B_array[i]);

#ifdef PRINT
  printf("A = [%.02f, %.02f, %.02f]\n"
         "    [%.02f, %.02f, %.02f]\n"
         "    [%.02f, %.02f, %.02f]\n"
         "    [%.02f, %.02f, %.02f]\n",
         args.A_array[i][0], args.A_array[i][4], args.A_array[i][8],
         args.A_array[i][1], args.A_array[i][5], args.A_array[i][9],
         args.A_array[i][2], args.A_array[i][6], args.A_array[i][10],
         args.A_array[i][3], args.A_array[i][7], args.A_array[i][11]);
 
  printf("B = [%.02f]\n"
         "    [%.02f]\n"
         "    [%.02f]\n"
         "    [%.02f]\n",
         args.B_array[i][0], args.B_array[i][1], args.B_array[i][2], args.B_array[i][3]);
#endif
}

__host__
void iterative_linear_ls_wrapper(dim3 const &b_p_g, dim3 const &t_p_b,
                                 IterativeLSArguments args,
                                 iteration_cb doLS, void *user_data)
{
  prepare_triangulation<<<b_p_g, t_p_b>>>(args);

  for (int i = 0; i < 10; i++) {
    // iterative solution
    prepare_iteration<<<b_p_g, t_p_b>>>(i, args);

    if (!doLS(args.A_array, args.B_array, args.n_keypoints, args.device_info, user_data)) {
      break;
    }

    recalc_weights<<<b_p_g, t_p_b>>>(i, args);
  }

  cleanup_triangulation<<<b_p_g, t_p_b>>>(args);
}
