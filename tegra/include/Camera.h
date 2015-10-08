#ifndef CAMERA_H_INCLUDED
#define CAMERA_H_INCLUDED

#include <opencv2/core.hpp>

class Camera {
private:
  cv::Mat mK;
  cv::Mat mKappa;
  size_t const mWidth;
  size_t const mHeight;

public:
  Camera(size_t w, size_t h);

  cv::Mat const &getCalibrationMatrix() const;
  cv::Mat const &getDistortionCoefficients() const;

  size_t getWidth() const;
  size_t getHeight() const;
};

#endif
