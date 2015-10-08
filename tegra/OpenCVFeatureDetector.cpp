#include "OpenCVFeatureDetector.h"

#include <iostream>
#include <set>

OpenCVFeatureDetector::~OpenCVFeatureDetector()
{
}

bool OpenCVFeatureDetector::findFeatures(Frame *frame)
{
  assert(!mDetector.empty());
  assert(!mExtractor.empty());

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

  mDetector->detect(frame->getImage(), keypoints);
  mExtractor->compute(frame->getImage(), keypoints, descriptors);

  frame->setKeypoints(keypoints);
  frame->setDescriptors(descriptors);

  return true;
}

ViewCombination OpenCVFeatureDetector::matchFrames(Frame *left, Frame *right)
{
  assert(!mMatcher.empty());
  ViewCombination view(left, right);

  std::vector<cv::DMatch> matches, good_matches;

  mMatcher->clear();
  mMatcher->add(left->getDescriptors());

  mMatcher->match(left->getDescriptors(), right->getDescriptors(), matches);

  filterMatches(matches, left->getKeypointsSize(), right->getKeypointsSize(), good_matches);

  view.setMatches(good_matches);

  return view;
}

