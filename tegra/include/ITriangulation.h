#ifndef ITRIANGULATION_H_INCLUDED
#define ITRIANGULATION_H_INCLUDED

#include <opencv2/core.hpp>

#include "Camera.h"
#include "ViewCombination.h"
#include "PointCloud.h"

class ITriangulation {

protected:
  template <typename T>
  cv::Point_<T> reprojectPoint(Camera const &cam, cv::Mat_<T> const &X, cv::Mat_<T> const &P);
  template <typename T>
  cv::Point_<T> distort(Camera const & cam, cv::Point_<T> const &pt);

public:
  ITriangulation();
  virtual ~ITriangulation();

  virtual bool triangulate(Camera const & cam, ViewCombination const &view, PointCloud &pts) = 0;

  template <typename T>
  static T norm(cv::Point_<T> const &x1, cv::Point_<T> const &x2);
};

template <typename T>
cv::Point_<T> ITriangulation::reprojectPoint(Camera const &cam, cv::Mat_<T> const &X,
                                             cv::Mat_<T> const &P)
{
  cv::Mat_<T> K = cam.getCalibrationMatrix();
  cv::Mat_<T> x_h = K * P * X;
  cv::Point_<T> x(x_h(0) / x_h(2), x_h(1) / x_h(2));
  cv::Point_<T> x_d = distort<T>(cam, x);

#ifdef PRINT
  std::cout << "CPU distorting x = " << x << std::endl;
  std::cout << " --> xd = " << x_d << std::endl;
#endif

  return x_d;
}

template <typename T>
cv::Point_<T> ITriangulation::distort(Camera const & cam, cv::Point_<T> const &pt)
{
  // convert point to normalized image coordinates
  T x = (pt.x * 2 - cam.getWidth()) / cam.getWidth();
  T y = (pt.y * 2 - cam.getHeight()) / cam.getHeight();
  T r = x * x + y * y;

  cv::Mat_<T> kappa = cam.getDistortionCoefficients();
  // radial distortion
  T radial = (1 + kappa(0) * pow(r, 2)
                + kappa(1) * pow(r, 4)
                + kappa(2) * pow(r, 6));

  // tangential distortion
  T d_x = 2 * kappa(3) * x * y + kappa(4) * (pow(r, 2) + 2 * pow(x, 2));
  T d_y = kappa(3) * (pow(r, 2) + 2 * pow(y, 2)) + 2 * kappa(4) * x * y;

  // normalized distorted image coordinates
  T u = x * radial + d_x;
  T v = y * radial + d_y;

  return cv::Point_<T>((u + 1) * cam.getWidth() / 2,
                       (v + 1) * cam.getHeight() / 2);
}

template <typename T>
T ITriangulation::norm(cv::Point_<T> const &x1, cv::Point_<T> const &x2)
{
  cv::Point_<T> diff = x1 - x2;
  return sqrt(diff.x * diff.x + diff.y * diff.y);
}

#endif
