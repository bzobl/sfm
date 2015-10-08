#ifndef OPENCV_FEATURE_DETECTOR_H_INCLUDED
#define OPENCV_FEATURE_DETECTOR_H_INCLUDED

#include <cassert>
#include <opencv2/features2d.hpp>

#include "IFeatureDetect.h"

class OpenCVFeatureDetector : public IFeatureDetect {

private:
  cv::Ptr<cv::FeatureDetector> mDetector;
  cv::Ptr<cv::DescriptorExtractor> mExtractor;
  cv::Ptr<cv::DescriptorMatcher> mMatcher;

public:
  virtual ~OpenCVFeatureDetector();

  template <typename TDetector, typename ...TArgs>
  void createDetector(TArgs const &...args) {
    mDetector = TDetector::create(args...);
  }

  template <typename TExtractor, typename ...TArgs>
  void createExtractor(TArgs const &...args) {
    mExtractor = TExtractor::create(args...);
  }

  template <typename TMatcher, typename ...TArgs>
  void createMatcher(TArgs const &...args) {
    mMatcher.reset(new TMatcher(args...));
  }

  virtual bool findFeatures(Frame *frame);
  virtual ViewCombination matchFrames(Frame *left, Frame *right);
};

#endif
