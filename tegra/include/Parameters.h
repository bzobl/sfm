#ifndef PARAMETERS_H_INCLUDED
#define PARAMETERS_H_INCLUDED

#include <opencv2/core.hpp>
//#include <options.h>

inline cv::Mat getK()
{
  static cv::Mat K = (cv::Mat_<double>(3, 3) << 922.45051,   0.00147, 635.39454,
                                                  0      , 920.41192, 354.20846,
                                                  0      ,   0      ,   1);
  return K;
}

inline cv::Mat getKappa()
{
  static cv::Mat kappa = (cv::Mat_<double>(1, 5) << 0.10620, -0.20464, 0, 0, 0);
  return kappa;
}

inline cv::Mat getImageDim(int width, int height)
{
  cv::Mat img_dim = (cv::Mat_<double>(1, 2) << width, height);
  return img_dim;
}

#endif
