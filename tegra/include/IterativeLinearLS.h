#ifndef ITERATIVE_LINEAR_LS_H_INCLUDED
#define ITERATIVE_LINEAR_LS_H_INCLUDED

#include "ITriangulation.h"
#include <iostream>

template <typename T>
class IterativeLinearLS : public ITriangulation {

private:
  const int MAX_ITERATIONS = 10;
  const T CONVERGE_THRESHOLD = 1e-9;
  //int solve_method = cv::DECOMP_SVD;
  int solve_method = cv::DECOMP_QR;

  cv::Mat linearLS(cv::Mat_<T> const &P_0, cv::Mat_<T> const &x_0c, T const w_0,
                   cv::Mat_<T> const &P_1, cv::Mat_<T> const &x_1c, T const w_1);
 
public:
  IterativeLinearLS();
  virtual ~IterativeLinearLS();

  virtual bool triangulate(Camera const &cam, ViewCombination const &view, PointCloud &pts);
};

template <typename T>
IterativeLinearLS<T>::IterativeLinearLS() : ITriangulation()
{
}

template <typename T>
IterativeLinearLS<T>::~IterativeLinearLS()
{
}

template <typename T>
cv::Mat IterativeLinearLS<T>::linearLS(cv::Mat_<T> const &P_0, cv::Mat_<T> const &x_0c, T const w_0,
                                       cv::Mat_<T> const &P_1, cv::Mat_<T> const &x_1c, T const w_1)
{
  T u0 = x_0c(0);
  T v0 = x_0c(1);
  T u1 = x_1c(0);
  T v1 = x_1c(1);
  
  // TODO extract A0 and A1 from the iteration
  cv::Mat_<T> A0 = (cv::Mat_<T>(2, 3) << P_0(0,0) - u0 * P_0(2,0), P_0(0,1) - u0 * P_0(2,1), P_0(0,2) - u0 * P_0(2,2),
                                     P_0(1,0) - v0 * P_0(2,0), P_0(1,1) - v0 * P_0(2,1), P_0(1,2) - v0 * P_0(2,2))
               * (1 / w_0);
  cv::Mat_<T> A1 = (cv::Mat_<T>(2, 3) << P_1(0,0) - u1 * P_1(2,0), P_1(0,1) - u1 * P_1(2,1), P_1(0,2) - u1 * P_1(2,2),
                                     P_1(1,0) - v1 * P_1(2,0), P_1(1,1) - v1 * P_1(2,1), P_1(1,2) - v1 * P_1(2,2))
               * (1 / w_1);
  cv::Mat_<T> A;
  cv::vconcat(A0, A1, A);
  
  cv::Mat_<T> B0 = (cv::Mat_<T>(2, 1) << u0 * P_0(2, 3) - P_0(0,3),
                                     v0 * P_0(2, 3) - P_0(1,3)) * (1 / w_0);
  cv::Mat_<T> B1 = (cv::Mat_<T>(2, 1) << u1 * P_1(2, 3) - P_1(0,3),
                                     v1 * P_1(2, 3) - P_1(1,3)) * (1 / w_1);
  cv::Mat_<T> B;
  cv::vconcat(B0, B1, B); 

#ifdef PRINT
    std::cout << "A = " << std::endl << A << std::endl;
    std::cout << "B = " << std::endl << B << std::endl;
#endif

  cv::Mat_<T> X;
  cv::solve(A, B, X, solve_method);
  X.resize(4, 1);

  return X;
}

template <typename T>
bool IterativeLinearLS<T>::triangulate(Camera const &cam, ViewCombination const &view, PointCloud &pts)
{
  cv::Mat_<T> K = cam.getCalibrationMatrix();
  cv::Mat_<T> K_inv;
  cv::invert(K, K_inv);
  cv::Mat_<T> P_0 = view.getImageLeft()->getProjectionMatrix();
  cv::Mat_<T> P_1 = view.getImageRight()->getProjectionMatrix();

  for (size_t match = 0; match < view.getMatchesSize(); match++) {
    cv::Mat_<T> x_0 = (cv::Mat_<T>(3, 1) << view.getMatchingPointLeft(match).x,
                                        view.getMatchingPointLeft(match).y,
                                        1);
    cv::Mat_<T> x_1 = (cv::Mat_<T>(3, 1) << view.getMatchingPointRight(match).x,
                                        view.getMatchingPointRight(match).y,
                                        1);
    cv::Mat_<T> x_0c = K_inv * x_0;
    cv::Mat_<T> x_1c = K_inv * x_1;
    cv::Mat_<T> X;

#ifdef PRINT
    std::cout << "normalized x = " << x_0.t() << " --> xc = " << x_0c.t() << std::endl;

    std::cout << "P_0 = " << P_0 << std::endl;
    std::cout << "P_1 = " << P_1 << std::endl;
#endif

    T w_0 = 1;
    T w_1 = 1;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      X = linearLS(P_0, x_0c, w_0,
                   P_1, x_1c, w_1);

#ifdef PRINT
      std::cout << "iteration " << i << " " << x_0c.t() <<  " --> X = " << X.t() << std::endl;
#endif

      // check if points as converged enough
      T w_0_new = cv::Mat(P_0.row(2) * X).at<T>(0);
      T w_1_new = cv::Mat(P_1.row(2) * X).at<T>(0);

      if ((std::fabs(w_0 - w_0_new) < CONVERGE_THRESHOLD) &&
          (std::fabs(w_1 - w_1_new) < CONVERGE_THRESHOLD)) {
        break;
      }

      w_0 = w_0_new;
      w_1 = w_1_new;
    }

    // reproject and compute error
    cv::Point_<T> x = reprojectPoint<T>(cam, X, P_1);
    T repr_error = norm(x, cv::Point_<T>(x_1(0), x_1(1)));

#ifdef PRINT
    std::cout << " triangulatd point: " << X.t() << " error: " << repr_error << std::endl;
#endif

    pts.addPoint(X, repr_error,
                 std::make_pair(view.getImageLeft(), view.getMatch(match).queryIdx),
                 std::make_pair(view.getImageRight(), view.getMatch(match).trainIdx));
  }

  return true;
}

#endif
