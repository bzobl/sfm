#ifndef IPOSE_ESTIMATION_H_INCLUDED
#define IPOSE_ESTIMATION_H_INCLUDED

#include <cassert>
#include <iostream>

#include <opencv2/core.hpp>
#include "ViewCombination.h"

#include "Camera.h"
#include "ITriangulation.h"
#include "PointCloud.h"

class IPoseEstimation {

private:
  double const CHIRALITY_THRESHOLD = 0.75;

protected:
  ITriangulation *mTriangulation;

  virtual bool testChirality(PointCloud const &pts, cv::Mat const &P);

public:
  IPoseEstimation(ITriangulation *triangulation);
  virtual ~IPoseEstimation();

  // estimate pose between two views for initial pose estimation
  virtual bool estimatePose(Camera const &cam, ViewCombination const &view,
                            cv::Mat &P_0, cv::Mat &P_1, PointCloud &pts) = 0;

  // estimate pose from known 3D-2D correspondences
  virtual bool estimatePose(Camera const &cam, Frame &frame, std::vector<cv::Point3f> const &known_3d,
                            std::vector<cv::Point2f> const &known_2d) = 0;
};

#endif
