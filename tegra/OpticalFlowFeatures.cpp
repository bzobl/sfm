#include "OpticalFlowFeatures.h"

#include <iostream>
#include <cassert>

#include <opencv2/optflow.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <opencv2/highgui.hpp>

#include "tests.h"
#include "ITriangulation.h"

OpticalFlowFeatures::~OpticalFlowFeatures()
{
}

void OpticalFlowFeatures::setFarnebackOptions(double pyr_scale, int levels, int winsize,
                                              int iterations, int poly_n, double sigma_n, int flags)
{
  mFarnPyrScale = pyr_scale;
  mFarnLevels = levels;
  mFarnWinsize = winsize;
  mFarnIterations = iterations;
  mFarnPolyN = poly_n;
  mFarnSigmaN = sigma_n;
  mFarnFlags = flags;
}

bool OpticalFlowFeatures::matchFarneback(ViewCombination &view)
{
  std::vector<cv::KeyPoint> kp_left;
  std::vector<cv::KeyPoint> kp_right;
  std::vector<cv::DMatch> matches;

  cv::Mat_<cv::Point2f> flow;
  cv::calcOpticalFlowFarneback(view.getImageLeft()->getImage(), view.getImageRight()->getImage(), flow,
                               mFarnPyrScale, mFarnLevels, mFarnWinsize, mFarnIterations, mFarnPolyN, mFarnSigmaN, mFarnFlags);
  int x_steps = 1;
  int y_steps = 1;

  size_t n_kpts = 0;

  // visualize
  cv::Mat result, result2;
  view.getImageLeft()->getColorImage().copyTo(result);
  view.getImageRight()->getColorImage().copyTo(result2);
  cv::imshow("Right", result2);
  std::cout << "starting farneback visualization" << std::endl;
  for (int y = 0; y < view.getImageLeft()->getHeight(); y += 10) {
    for (int x = 0; x < view.getImageLeft()->getWidth(); x += 10) {
      //double dx = flowx.at<float>(y, x);
      //double dy = flowy.at<float>(y, x);
      double dx = flow.at<cv::Point2f>(y, x).x;
      double dy = flow.at<cv::Point2f>(y, x).y;

      double l = std::sqrt(dx*dx + dy*dy);
      std::cout << "l=" << l << std::endl;

      if ((l > 0)) {
        cv::Point p1(x, y);
        cv::Point p2(x + dx, y + dy);

        std::cout << "drawing line" << std::endl;
        cv::arrowedLine(result, p1, p2, cv::Scalar(255, 255, 255));
      }
    }
  }
  cv::imshow("Farneback", result);
  while (cv::waitKey(30) != 'q') { }

  /*
  size_t kpts_in_right = view.getImageRight()->getKeypointsSize();

  for (int i = 0 ; i < view.getImageLeft()->getKeypointsSize(); i++) {
    cv::Point2f const &pt = view.getImageLeft()->getImagepoint(i);

    const cv::Point2f &offset = flow.at<cv::Point2f>(pt.y, pt.x);
    cv::Point2f kp2(cvRound(pt.x + offset.x), cvRound(pt.y + offset.y));

    //std::cout << "left = " << pt << ", right = " << kp2 << ", offset = " << offset << std::endl;

    double distance = ITriangulation::norm(pt, kp2);

    view.getImageRight()->addKeypoint(cv::KeyPoint(kp2, 1));
    matches.emplace_back(kpts_in_right, i, distance);
    kpts_in_right++;
  }

  view.setMatches(matches);
  //test_show_features_iteratively(view);
  return true;
  */

  for (size_t w = 0; w < view.getImageLeft()->getWidth(); w += x_steps) {
    for (size_t h = 0; h < view.getImageLeft()->getHeight(); h += y_steps) {

      const cv::Point2f &offset = flow.at<cv::Point2f>(h, w);
      cv::Point2f kp1(w, h);
      cv::Point2f kp2(cvRound(kp1.x + offset.x), cvRound(kp1.y + offset.y));

      if (   (kp2.x >= view.getImageRight()->getWidth())
          || (kp2.y >= view.getImageRight()->getHeight())
          || (kp2.x < 0) || (kp2.y < 0)) {
        continue;
      }

      double distance = ITriangulation::norm(kp1, kp2);

      if (offset.x < 1 && offset.y < 1) continue;

      if ((distance <= 10.0) || (distance >= 380)) {
        continue;
      }

      //std::cout << kp1 << "-->" << kp2 << " = " << distance << std::endl;

      matches.emplace_back(n_kpts, n_kpts, distance);
      n_kpts++;
      kp_left.emplace_back(kp1, 1);
      kp_right.emplace_back(kp2, 1);
      //std::cout << " " << n_kpts << " inserted" << std::endl;
    }
  }
  std::cout << n_kpts << " keypoints" << std::endl;

  test_show_features_iteratively(view);

  view.getImageLeft()->setKeypoints(kp_left);
  view.getImageRight()->setKeypoints(kp_right);
  view.setMatches(matches);
  return true;
}

bool OpticalFlowFeatures::matchKL(ViewCombination &view)
{
  std::vector<cv::Point2f> imgp_left;
  std::vector<cv::Point2f> imgp_right;
  std::vector<cv::KeyPoint> kp_left;
  std::vector<cv::KeyPoint> kp_right;
  std::vector<cv::DMatch> matches;
  std::vector<uchar> status;
  std::vector<float> err;

  std::cout << "searching features" <<std::endl;
  //cv::goodFeaturesToTrack(view.getImageLeft()->getImage(), imgp_left, 500, 0.01, 15);
  cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SURF::create();
  detector->detect(view.getImageLeft()->getImage(), kp_left);
  cv::KeyPoint::convert(kp_left, imgp_left);
  kp_left.clear();

  cv::Mat left, right;
  view.getImageLeft()->getImage().convertTo(left, CV_8UC1);
  view.getImageRight()->getImage().convertTo(right, CV_8UC1);

  cv::calcOpticalFlowPyrLK(view.getImageLeft()->getImage(), view.getImageRight()->getImage(),
                           //left, right,
                           imgp_left, imgp_right,
                           status, err,
                           mKLWinsize, mKLLeves);

  for (size_t i = 0; i < status.size(); i++) {

    if (status[i] != 1) {
      continue;
    }

    /*
    double distance = cv::norm(imgp_right[i] - imgp_left[i]);

    if (distance <= 5) {
      continue;
    }
    */

    std::cout << imgp_left[i] << "-->" << imgp_right[i] << " = " << err[i] << std::endl;

    matches.emplace_back(kp_left.size(), kp_right.size(), 1);
    kp_left.emplace_back(imgp_left[i], 1);
    kp_right.emplace_back(imgp_right[i], 1);
  }

  std::cout << matches.size() << " keypoints inserted" << std::endl;

  view.getImageLeft()->setKeypoints(kp_left);
  view.getImageRight()->setKeypoints(kp_right);
  view.setMatches(std::move(matches));
  return true;
}

bool OpticalFlowFeatures::findFeatures(Frame *frame)
{
  assert(false);
}

ViewCombination OpticalFlowFeatures::matchFrames(Frame *left, Frame *right)
{
  ViewCombination view(left, right);
  bool success = false;

  switch (mMethod) {
    case OF_METHOD_FARNEBACK:
      success = matchFarneback(view);
      break;
    case OF_METHOD_KL:
      success = matchKL(view);
      break;
    default:
      assert(false);
      break;
  }

  return view;
}
