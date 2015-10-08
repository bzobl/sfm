#include "GPUSurf.h"

#include <opencv2/cuda.hpp>
#include <opencv2/cudafeatures2d.hpp>

GPUSurf::GPUSurf(double hessian, int octaves, int layers, bool extended, bool upright,
                 int norm, float ratio)
                : mSurf(hessian, octaves, layers, extended, ratio, upright), mNorm(norm)
{
}

bool GPUSurf::findFeatures(Frame *frame)
{
  cv::cuda::GpuMat gpu_image(&mImageAllocator);
  cv::cuda::GpuMat gpu_keypoints(&mKeypointsAllocator);
  cv::cuda::GpuMat gpu_mask(&mMaskAllocator);

  gpu_image.upload(frame->getImage());
  mSurf(gpu_image, gpu_mask, gpu_keypoints, frame->getGpuDescriptors());

  std::vector<cv::KeyPoint> keypoints;
  mSurf.downloadKeypoints(gpu_keypoints, keypoints);
  frame->setKeypoints(keypoints);

  return true;
}

ViewCombination GPUSurf::matchFrames(Frame *left, Frame *right)
{
  cv::cuda::BFMatcher_CUDA matcher(mNorm);
  std::vector<cv::DMatch> matches, good_matches;
  ViewCombination view(left, right);

  matcher.match(left->getGpuDescriptors(), right->getGpuDescriptors(), matches);

  filterMatches(matches, left->getKeypointsSize(), right->getKeypointsSize(), good_matches);

  view.setMatches(good_matches);
  return view;
}
