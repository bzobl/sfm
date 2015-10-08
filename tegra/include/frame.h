#ifndef FRAME_H_INCLUDED
#define FRAME_H_INCLUDED

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/cuda.hpp>

class Frame {

private:
  cv::Mat mColorImage;
  cv::Mat mImage;

  cv::Mat mP;
  std::vector<cv::KeyPoint> mKeypoints;
  cv::Mat mDescriptors;

  cv::cuda::GpuMat mGpuDescriptors;

public:
  Frame(cv::Mat const &image);

  cv::Mat const getColorImage() const;
  cv::Mat const getImage() const;
  size_t getWidth() const;
  size_t getHeight() const;

  cv::Mat getProjectionMatrix() const;
  void setProjectionMatrix(cv::Mat const &P);

  cv::KeyPoint const &getKeypoint(size_t i) const;
  size_t getKeypointsSize() const;
  std::vector<cv::KeyPoint> getKeypoints() const;
  void setKeypoints(std::vector<cv::KeyPoint> const &kp);
  void addKeypoint(cv::KeyPoint const &kp);

  cv::Point2f const &getImagepoint(size_t i) const;
  std::vector<cv::Point2f> getImagepoints() const;

  cv::Mat const &getDescriptors() const;
  void setDescriptors(cv::Mat const &desc);

  cv::cuda::GpuMat &getGpuDescriptors();

  cv::Mat drawKeypoints() const;
  cv::Mat drawKeypoints(std::vector<cv::KeyPoint> keypoints) const;
  cv::Mat drawKeypoints(std::vector<cv::Point2f> points) const;
};

#endif
