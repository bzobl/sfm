#include "ViewCombination.h"

ViewCombination::ViewCombination(Frame *left, Frame *right)
                                : mLeft(left), mRight(right)
{
}

Frame *ViewCombination::getImageLeft() const
{
  return mLeft;
}

Frame *ViewCombination::getImageRight() const
{
  return mRight;
}

void ViewCombination::setPointCloud(PointCloud * cloud)
{
  mCloud = cloud;
}

PointCloud *ViewCombination::getPointCloud() const
{
  return mCloud;
}

cv::Point2f ViewCombination::getMatchingPointLeft(size_t i) const
{
  return mLeft->getImagepoint(mMatches.at(i).queryIdx);
}

cv::Point2f ViewCombination::getMatchingPointRight(size_t i) const
{
  return mRight->getImagepoint(mMatches.at(i).trainIdx);
}

std::vector<cv::Point2f> ViewCombination::getMatchingPointsLeft() const
{
  std::vector<cv::Point2f> pts;
  pts.resize(mMatches.size());
  for (size_t i = 0; i < mMatches.size(); i++) {
    pts[i] = getMatchingPointLeft(i);
  }
  return pts;
}

std::vector<cv::Point2f> ViewCombination::getMatchingPointsRight() const
{
  std::vector<cv::Point2f> pts;
  pts.resize(mMatches.size());
  for (size_t i = 0; i < mMatches.size(); i++) {
    pts[i] = getMatchingPointRight(i);
  }
  return pts;
}

void ViewCombination::setMatches(std::vector<cv::DMatch> const &matches)
{
  mMatches = matches;
}

size_t ViewCombination::getMatchesSize() const
{
  return mMatches.size();
}

cv::DMatch const &ViewCombination::getMatch(size_t i) const
{
  return mMatches.at(i);
}

std::vector<cv::DMatch> ViewCombination::getMatches() const
{
  return mMatches;
}
