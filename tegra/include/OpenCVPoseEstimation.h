#ifndef OPENCV_POSE_ESTIMATION_H_INCLUDED
#define OPENCV_POSE_ESTIMATION_H_INCLUDED

#include "IPoseEstimation.h"
#include "ITriangulation.h"

#include <opencv2/calib3d.hpp>

class OpenCVPoseEstimation : public IPoseEstimation {

private:
  int mFindFMethod = cv::FM_RANSAC;
  double mFindFOutlierDistance = 3.0;
  double mFindFConfidence = 0.99;

  double DET_E_THRESHOLD = 1e-07;
  double SING_RATIO_E_THRESHOLD = 0.3;
  double DET_ROT_MAT_THRESHOLD = 1e-09;

  double AVG_REPROJECTION_ERROR_THRESHOLD = 200;

  inline bool isRotationMatrix(cv::Mat const &R);
  inline bool checkE(cv::Mat const &E, cv::Mat const &svd_sigma);

public:
  OpenCVPoseEstimation(ITriangulation *triangulation);
  virtual ~OpenCVPoseEstimation();

  void setFindFParameter(int method, double param_1 = 3.0, double param_2 = 0.99);

  virtual bool estimatePose(Camera const &cam, ViewCombination const &view,
                            cv::Mat &P_0, cv::Mat &P_1, PointCloud &pts);

  virtual bool estimatePose(Camera const &cam, Frame &frame, std::vector<cv::Point3f> const &known_3d,
                            std::vector<cv::Point2f> const &known_2d);
};

#endif
