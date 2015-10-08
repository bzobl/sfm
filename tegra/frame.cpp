#include "frame.h"

#include <cassert>
#include <opencv2/imgproc.hpp>


Frame::Frame(cv::Mat const &image)
{
  image.copyTo(mColorImage);
  cv::cvtColor(mColorImage, mImage, cv::COLOR_BGR2GRAY);
}

cv::Mat const Frame::getColorImage() const
{
  return mColorImage;
}

cv::Mat const Frame::getImage() const
{
  return mImage;
}

size_t Frame::getWidth() const
{
  return mImage.cols;
}

size_t Frame::getHeight() const
{
  return mImage.rows;
}

cv::Mat Frame::getProjectionMatrix() const
{
  return mP;
}

void Frame::setProjectionMatrix(cv::Mat const &P)
{
  assert(P.cols == 4);
  assert(P.rows == 3);

  P.copyTo(mP);
}

cv::Point2f const &Frame::getImagepoint(size_t i) const
{
  return mKeypoints.at(i).pt;
}

std::vector<cv::Point2f> Frame::getImagepoints() const
{
  std::vector<cv::Point2f> pts;
  cv::KeyPoint::convert(mKeypoints, pts);
  return pts;
}

cv::KeyPoint const &Frame::getKeypoint(size_t i) const
{
  return mKeypoints.at(i);
}

size_t Frame::getKeypointsSize() const
{
  return mKeypoints.size();
}

std::vector<cv::KeyPoint> Frame::getKeypoints() const
{
  return mKeypoints;
}

void Frame::setKeypoints(std::vector<cv::KeyPoint> const &kp)
{
  mKeypoints = kp;
}

void Frame::addKeypoint(cv::KeyPoint const &kp)
{
  mKeypoints.push_back(kp);
}

cv::Mat const &Frame::getDescriptors() const
{
  return mDescriptors;
}

void Frame::setDescriptors(cv::Mat const &desc)
{
  desc.copyTo(mDescriptors);
}

cv::cuda::GpuMat &Frame::getGpuDescriptors()
{
  return mGpuDescriptors;
}

cv::Mat Frame::drawKeypoints() const
{
  return drawKeypoints(mKeypoints);
}

cv::Mat Frame::drawKeypoints(std::vector<cv::KeyPoint> keypoints) const
{
  std::vector<cv::Point2f> pts;
  cv::KeyPoint::convert(keypoints, pts);
  return drawKeypoints(pts);
}

cv::Mat Frame::drawKeypoints(std::vector<cv::Point2f> points) const
{
  cv::Mat img;
  mColorImage.copyTo(img);
  cv::RNG rng;

  for (cv::Point2f pt : points) {
    cv::Scalar color(rng(256), rng(256), rng(256));
    //cv::circle(img, pt, 7, cv::Scalar(0, 0x1F, 0x1F));
    cv::circle(img, pt, 7, color, cv::FILLED);
  }

  return img;
}
