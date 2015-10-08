#ifndef IFEATURE_DETECT_H_INCLUDED
#define IFEATURE_DETECT_H_INCLUDED

#include <set>

#include "frame.h"
#include "ViewCombination.h"

class IFeatureDetect
{
protected:

  void filterMatches(std::vector<cv::DMatch> const &matches,
                     int n_keypoints_left, int n_keypoints_right,
                     std::vector<cv::DMatch> &good_matches)
  {
    std::set<size_t> existing_trainIdx;

    good_matches.clear();

    for (size_t i = 0; i < matches.size(); i++) {
      if ((existing_trainIdx.find(matches[i].trainIdx) == existing_trainIdx.end())
          && (matches[i].queryIdx >= 0)
          && (matches[i].queryIdx < n_keypoints_left)
          && (matches[i].trainIdx >= 0)
          && (matches[i].trainIdx < n_keypoints_right)) {

        good_matches.push_back(matches[i]);
        existing_trainIdx.insert(matches[i].trainIdx);
      }
    }
  }

public:
  virtual ~IFeatureDetect() {}

  virtual bool findFeatures(Frame *frame) = 0;
  virtual ViewCombination matchFrames(Frame *left, Frame *right) = 0;
};

#endif
