#include "IPoseEstimation.h"

#include <cmath>

IPoseEstimation::IPoseEstimation(ITriangulation *triang) : mTriangulation(triang)
{
}

IPoseEstimation::~IPoseEstimation()
{
}

bool IPoseEstimation::testChirality(PointCloud const &pts, cv::Mat const &P)
{
  size_t processed_points = 0;
  size_t in_view = 0;
  for (size_t i = 0; i < pts.size(); i++) {
    if (pts.getPoint(i)(3) <= 0) {
      continue;
    }
    processed_points++;

    cv::Mat pt_2d = P * cv::Mat(pts.getPoint(i));

    if (std::isnan(pt_2d.at<double>(0)) || std::isnan(pt_2d.at<double>(1))) {
      continue;
    }

    // check weight of point, if it is > 0 the point is in front of the camera
    if (pt_2d.at<double>(2) > 0) {
      in_view++;
    }
  }

  if (((double)in_view / processed_points) < CHIRALITY_THRESHOLD) {
#ifdef POSE_PRINT
    std::cout << "only " << in_view << "/" << processed_points
              << " (of " << pts.size() << ") points are in front of the camera" << std::endl;
#endif
    return false;
  }

  return true;
}
