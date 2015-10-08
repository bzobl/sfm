#ifndef RUBIK_MANUAL_FEATURES_H_INCLUDED
#define RUBIK_MANUAL_FEATURES_H_INCLUDED

#include "IFeatureDetect.h"

class RubikManualFeatures : public IFeatureDetect {
private:

  std::vector<cv::Point2f> getRubikPoints(size_t idx) const;
  std::vector<cv::Point2f> getRubik1Points() const;
  std::vector<cv::Point2f> getRubik2Points() const;
  std::vector<cv::Point2f> getRubik3Points() const;
  std::vector<cv::Point2f> getRubik4Points() const;

  std::map<Frame *, size_t> mRubikIndices;

public:
  virtual ~RubikManualFeatures();

  void preloadFrame(Frame *frame, size_t rub_idx);

  virtual bool findFeatures(Frame *frame);
  virtual ViewCombination matchFrames(Frame *left, Frame *right);
};

#endif
