#ifndef VIEW_COMBINATION_H_INCLUDED
#define VIEW_COMBINATION_H_INCLUDED

#include "frame.h"
#include "PointCloud.h"

class ViewCombination {

private:
  Frame *mLeft;
  Frame *mRight;

  PointCloud *mCloud;

  enum {
    LEFT_IMG = 0,
    RIGHT_IMG = 1,
  };

  std::vector<cv::DMatch> mMatches;

public:
  ViewCombination() = default;
  ViewCombination(Frame *left, Frame *right);

  Frame *getImageLeft() const;
  Frame *getImageRight() const;

  void setPointCloud(PointCloud * cloud);
  PointCloud *getPointCloud() const;

  cv::Point2f getMatchingPointLeft(size_t i) const;
  cv::Point2f getMatchingPointRight(size_t i) const;

  std::vector<cv::Point2f> getMatchingPointsLeft() const;
  std::vector<cv::Point2f> getMatchingPointsRight() const;

  void setMatches(std::vector<cv::DMatch> const &matches);
  cv::DMatch const &getMatch(size_t i) const;
  size_t getMatchesSize() const;
  std::vector<cv::DMatch> getMatches() const;
};

#endif
