#ifndef OPTICALFLOW_FEATURES_H_INCLUDED
#define OPTICALFLOW_FEATURES_H_INCLUDED

#include "IFeatureDetect.h"

class OpticalFlowFeatures : public IFeatureDetect {

public:
  enum OFMethod {
    OF_METHOD_FARNEBACK,
    OF_METHOD_KL,
  };

private:
  OFMethod mMethod = OF_METHOD_FARNEBACK;

  bool matchFarneback(ViewCombination &view);
  bool matchKL(ViewCombination &view);

  // Farneback options
  double mFarnPyrScale = 0.5;
  int mFarnLevels = 1;
  int mFarnWinsize = 13;
  int mFarnIterations = 1;
  int mFarnPolyN = 7;
  double mFarnSigmaN = 1.5;
  int mFarnFlags = 0;

  // KL options
  cv::Size mKLWinsize = cv::Size(21, 21);
  int mKLLeves = 3;

public:
  virtual ~OpticalFlowFeatures();

  void setOpticalFlowMethod(OFMethod method);
  void setFarnebackOptions(double pyr_scale, int levels, int winsize, int iterations, int poly_n,
                           double sigma_n, int flags);

  virtual bool findFeatures(Frame *frame);
  virtual ViewCombination matchFrames(Frame *left, Frame *right);
};

#endif
