#include "tests.h"

#include <string>
#include <iostream>
#include <cassert>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc.hpp>

#include <cuda_runtime.h>
#include <cublas_v2.h>

#include "GPUHelper.h"
#include "GPUSurf.h"
#include "GPUIterativeLinearLS.h"
#include "OpenCVPoseEstimation.h"
#include "OpticalFlowFeatures.h"
#include "IterativeLinearLS.h"
#include "RubikManualFeatures.h"
#include "OpenCVFeatureDetector.h"
#include "FileWriter.h"
#include "StopWatch.h"

void test_feature_matching_all(Frame *left, Frame *right)
{
  OpenCVFeatureDetector *ocvfeat = new OpenCVFeatureDetector();
  ocvfeat->createMatcher<cv::BFMatcher>();
  ocvfeat->createMatcher<cv::BFMatcher>(cv::NORM_L2, true);
  //ocvfeat->createMatcher<cv::FlannBasedMatcher>();
  FileWriter fw("featuretest_keypoints.m");

  std::string detector_name, extractor_name;

  for (int detector = 0; detector != -1; detector++) {
    switch(detector) {
      case 0:
        ocvfeat->createDetector<cv::AKAZE>();
        detector_name = "AKAZE";
        break;
      case 1:
        ocvfeat->createDetector<cv::BRISK>();
        detector_name = "BRISK";
        break;
      case 2:
        ocvfeat->createDetector<cv::FastFeatureDetector>();
        detector_name = "FastFeatureDetector";
        break;
      case 3:
        ocvfeat->createDetector<cv::FastFeatureDetector>();
        detector_name = "FastFeatureDetector";
        break;
      case 4:
        ocvfeat->createDetector<cv::GFTTDetector>(300, 0.1, 1, 3, false, 0.04);
        detector_name = "GFTTDetector";
        break;
      case 5:
        ocvfeat->createDetector<cv::KAZE>();
        detector_name = "KAZE";
        break;
      case 6:
        ocvfeat->createDetector<cv::MSER>();
        detector_name = "MSER";
        break;
      case 7:
        ocvfeat->createDetector<cv::ORB>();
        detector_name = "ORB";
        break;
      case 8:
        ocvfeat->createDetector<cv::SimpleBlobDetector>();
        detector_name = "SimpleBlobDetector";
        break;
      case 9:
        ocvfeat->createDetector<cv::xfeatures2d::SIFT>();
        detector_name = "SIFT";
        break;
      case 10:
        ocvfeat->createDetector<cv::xfeatures2d::StarDetector>();
        detector_name = "StarDetector";
        break;
      case 11:
        ocvfeat->createDetector<cv::xfeatures2d::SURF>(500, 5, 5, true, true);
        detector_name = "SURF";
        break;
      default:
        detector = -1;
        break;
    }

    if (detector == -1) break;

    for (int extractor = 0; extractor != -1; extractor++) {
      switch(extractor) {
        case 0:
          ocvfeat->createExtractor<cv::AKAZE>();
          extractor_name = "AKAZE";
          break;
        case 1:
          ocvfeat->createExtractor<cv::BRISK>();
          extractor_name = "BRISK";
          break;
        case 2:
          ocvfeat->createExtractor<cv::KAZE>();
          extractor_name = "KAZE";
          break;
        case 3:
          ocvfeat->createExtractor<cv::ORB>();
          extractor_name = "ORB";
          break;
        case 4:
          ocvfeat->createExtractor<cv::xfeatures2d::BriefDescriptorExtractor>();
          extractor_name = "BriefDescriptorExtractor";
          break;
        case 5:
          ocvfeat->createExtractor<cv::xfeatures2d::FREAK>();
          extractor_name = "FREAK";
          break;
        case 6:
          ocvfeat->createExtractor<cv::xfeatures2d::SIFT>();
          extractor_name = "SIFT";
          break;
        case 7:
          ocvfeat->createExtractor<cv::xfeatures2d::SURF>(500, 5, 10, true, true);
          extractor_name = "SURF";
          break;
        default:
          extractor = -1;
          break;
      }

      if (extractor == -1) break;

      std::cout << "--- Detector " << detector_name << " | Extractor " << extractor_name << std::endl;

      try {
        ocvfeat->findFeatures(left);
        ocvfeat->findFeatures(right);
        ViewCombination view = ocvfeat->matchFrames(left, right);
        size_t n_features = view.getMatchesSize();
        std::cout << "    Found " << n_features <<" keypoints" << std::endl;

        if (n_features > 0) {
          test_show_features_colored(view, 30);
          //test_show_features_iteratively(view);
          /*
          fw.writePoints("kp_"+detector_name+"_"+extractor_name+"_1", view.getMatchingPointsLeft());
          fw.writePoints("kp_"+detector_name+"_"+extractor_name+"_2", view.getMatchingPointsRight());
          */
        }
      } catch (...) {
        std::cout << "Exception thrown" << std::endl;
      }
    }
  }
}

struct SurfSurfData {
  Frame *left;
  Frame *right;
  OpenCVFeatureDetector *ocvfeat;
  int hessian = 100;
  int n_octaves = 4;
  int oct_layer = 3;
  int extended = 0;
  int upright = 0;
  int distance = 1;
};
void on_surf_surf_change(int value, void *data)
{
  double start = cv::getTickCount();
  SurfSurfData *ssd = (SurfSurfData *)data;
  ssd->ocvfeat->createDetector<cv::xfeatures2d::SURF>(ssd->hessian, ssd->n_octaves, ssd->oct_layer,
                                                      ssd->extended != 0, ssd->upright != 0);
  ssd->ocvfeat->createExtractor<cv::xfeatures2d::SURF>(ssd->hessian, ssd->n_octaves, ssd->oct_layer,
                                                      ssd->extended != 0, ssd->upright != 0);
  ssd->ocvfeat->findFeatures(ssd->left);
  ssd->ocvfeat->findFeatures(ssd->right);
  ViewCombination view = ssd->ocvfeat->matchFrames(ssd->left, ssd->right);

  double stop = cv::getTickCount();
  std::cout << "found " << view.getMatchesSize() << " features in "
            << (stop - start) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
  cv::imshow("Left", view.getImageLeft()->drawKeypoints(view.getMatchingPointsLeft()));
  cv::imshow("Right", view.getImageRight()->drawKeypoints(view.getMatchingPointsRight()));
}

void test_feature_matching_interactive(Frame *left, Frame *right)
{
  OpenCVFeatureDetector *ocvfeat = new OpenCVFeatureDetector();
  ocvfeat->createMatcher<cv::BFMatcher>();
  FileWriter fw("featuretest_keypoints.m");

  std::string detector_name, extractor_name;

  enum {
    KAZE_KAZE,
    ORB_SURF,
    SURF_BRIEF,
    SURF_SURF,
  };
  int types = SURF_SURF;
  std::string win_name = "trackbars";
  cv::namedWindow(win_name, 1);

  switch (types) {
    case KAZE_KAZE:
      break;
    case ORB_SURF:
      break;
    case SURF_BRIEF:
      break;
    case SURF_SURF:
      {

      SurfSurfData *data = new SurfSurfData;
      data->left = left;
      data->right = right;
      data->ocvfeat = ocvfeat;
      cv::createTrackbar("hessian", win_name, &data->hessian, 1000, NULL, NULL);
      cv::createTrackbar("octaves", win_name, &data->n_octaves, 20, NULL, NULL);
      cv::createTrackbar("octave layers", win_name, &data->oct_layer, 10, NULL, NULL);
      cv::createTrackbar("extended", win_name, &data->extended, 1, NULL, NULL);
      cv::createTrackbar("upright", win_name, &data->upright, 1, NULL, NULL);
      cv::createTrackbar("distance", win_name, &data->distance, 1000, NULL, NULL);
      cv::createTrackbar("apply", win_name, NULL, 1, on_surf_surf_change, data);
      on_surf_surf_change(0, data);

      break;
      }
    default:
      assert(false);
      break;
  }

  while (true) {
    char key = cv::waitKey(50);

    if (key == 'q') break;
  }

}

void test_show_features_colored(ViewCombination const& view, size_t n, bool destroy)
{
  cv::Mat img_left, img_right;
  view.getImageLeft()->getColorImage().copyTo(img_left);
  view.getImageRight()->getColorImage().copyTo(img_right);

  std::cout << "-- " << view.getMatchesSize() << " Matches" << std::endl;

  cv::RNG rng;

  for (size_t i = 0; i < n; i++) {
    cv::Scalar color(rng(256), rng(256), rng(256));

    /*
    std::cout << "Matches: tidx=" << view.getMatch(i).queryIdx << ", qidx="
                                  << view.getMatch(i).trainIdx << ", distance="
                                  << view.getMatch(i).distance << ", imgidx="
                                  << view.getMatch(i).imgIdx << std::endl;
                                  */

    cv::circle(img_left, view.getMatchingPointLeft(i), 7, color, cv::FILLED);
    cv::circle(img_right, view.getMatchingPointRight(i), 7, color, cv::FILLED);
  }

  while (true) {
    cv::imshow("Matches, colored LEFT", img_left);
    cv::imshow("Matches, colored RIGHT", img_right);

    char key = cv::waitKey(30);
    if (key == 'q') {
      break;
    }
  }

  if (destroy) {
    cv::destroyWindow("Matches, colored LEFT");
    cv::destroyWindow("Matches, colored RIGHT");
  }
}

void test_show_features_iteratively(ViewCombination const& view)
{
  cv::Mat img_left, img_right;

  std::cout << "-- " << view.getMatchesSize() << " Matches" << std::endl;

  for (size_t i = 0; i < view.getMatchesSize(); i++) {
    std::cout << "Matches: tidx=" << view.getMatch(i).queryIdx << ", qidx="
                                  << view.getMatch(i).trainIdx << ", distance="
                                  << view.getMatch(i).distance << ", imgidx="
                                  << view.getMatch(i).imgIdx << std::endl;
    view.getImageLeft()->getColorImage().copyTo(img_left);
    view.getImageRight()->getColorImage().copyTo(img_right);

    cv::circle(img_left, view.getMatchingPointLeft(i), 7, cv::Scalar(0xFF, 0xFF, 0xFF), cv::FILLED);
    cv::circle(img_right, view.getMatchingPointRight(i), 7, cv::Scalar(0xFF, 0xFF, 0xFF), cv::FILLED);

    std::cout << "left = " << view.getMatchingPointLeft(i)
              << ", right = " << view.getMatchingPointRight(i)
              << std::endl;

    while (true) {
      cv::imshow("left", img_left);
      cv::imshow("right", img_right);

      char key = cv::waitKey(30);
      if (key == 'n') break;
      if (key == 'q') {
        return;
      }
    }
  }
}

ViewCombination time_feature_detection(std::string const &name, double hessian, int octaves,
                                       int layers, bool extended, bool upright,
                                       Frame *left, Frame *right,
                                       std::vector<cv::Point2f> &kp_left,
                                       std::vector<cv::Point2f> &kp_right,
                                       size_t n_runs = 10)
{
  StopWatch sw;
  IFeatureDetect *fdetect = nullptr;
  ViewCombination view;
  double creation = 0;
  double find_left = 0;
  double find_right = 0;
  double match = 0;

  sw.start();
  if (name == "CPU") {
    OpenCVFeatureDetector *cpu = new OpenCVFeatureDetector;
    cpu->createExtractor<cv::xfeatures2d::SURF>(hessian, octaves, layers, extended, upright);
    cpu->createDetector<cv::xfeatures2d::SURF>(hessian, octaves, layers, extended, upright);
    cpu->createMatcher<cv::BFMatcher>(cv::NORM_L2, true);
    fdetect = cpu;
  } else if (name == "GPU") {
    fdetect = new GPUSurf(hessian, octaves, layers, extended, upright, cv::NORM_L2);
  } else {
    assert(false);
  }
  sw.stop();
  creation += sw.getDurationMs();

  for (size_t r = 0; r < n_runs; r++) {
    sw.start();
    fdetect->findFeatures(left);
    sw.stop();
    find_left += sw.getDurationMs();

    sw.start();
    fdetect->findFeatures(right);
    sw.stop();
    find_right += sw.getDurationMs();

    sw.start();
    view = fdetect->matchFrames(left, right);
    sw.stop();
    match += sw.getDurationMs();
  }

  delete fdetect;

  find_left  /= n_runs;
  find_right /= n_runs;
  match      /= n_runs;

  std::cout << "create      " << creation << std::endl
            << "find left:  " << find_left << std::endl
            << " -> " << left->getKeypointsSize() << std::endl
            << "find right: " << find_right << std::endl
            << " -> " << right->getKeypointsSize() << std::endl
            << "match:      " << match << std::endl
            << " -> " << view.getMatchesSize() << std::endl;

  kp_left = left->getImagepoints();
  kp_right = right->getImagepoints();
  return view;
}

template <typename T>
int compare_points(std::vector<cv::Point_<T>> const &kp_left,
                   std::vector<cv::Point_<T>> const &kp_right,
                   std::map<size_t, size_t> *kp_map = nullptr)
{
  std::set<size_t> taken_matches;
  size_t errors = 0;

  for (size_t i = 0; i < kp_left.size(); i++) {
    double minimal_idx = -1;
    double minimal_norm = -1;

    // check distance to each point
    for (size_t j = 0; j < kp_right.size(); j++) {

      if (taken_matches.find(j) == taken_matches.end()) {
        double norm = ITriangulation::norm<double>(kp_left[i], kp_right[j]);
        if ((minimal_idx== -1) || (norm < minimal_norm)) {
          minimal_norm = norm;
          minimal_idx = j;
        }
      }
    }

    if ((minimal_idx != -1) && (minimal_norm < 10e-4)) {
      taken_matches.insert(minimal_idx);
      if (kp_map != nullptr) {
        //std::cout << "inserting " << i << " -> " << minimal_idx << " norm = " << minimal_norm << std::endl;
        //std::cout << "  " << kp_left[i] << " -> " << kp_right[minimal_idx] << std::endl;
        kp_map->insert(std::make_pair(i, minimal_idx));
      }
    } else {
      //std::cout << "nearest match for " << kp_left[i] << " --> " << kp_right[minimal_idx] << std::endl;
      errors++;
    }
  }
  return errors;
}

int compare_matches(ViewCombination const &vcpu, ViewCombination const &vgpu)
{
  std::map<size_t, size_t> kp_left_map;
  std::map<size_t, size_t> kp_right_map;

  compare_points(vcpu.getImageLeft()->getImagepoints(), vgpu.getImageLeft()->getImagepoints(),
                 &kp_left_map);
  compare_points(vcpu.getImageRight()->getImagepoints(), vgpu.getImageRight()->getImagepoints(),
                 &kp_right_map);

  size_t errors = 0;
  for (size_t i = 0; i < vcpu.getMatchesSize(); i++) {
    int cpu_left_idx = vcpu.getMatch(i).queryIdx;
    int cpu_right_idx = vcpu.getMatch(i).trainIdx;

    auto iter_left = kp_left_map.find(cpu_left_idx);
    auto iter_right = kp_right_map.find(cpu_right_idx);
    if ((iter_left == kp_left_map.end()) || (iter_right == kp_right_map.end())) {
      errors++;
      //std::cerr << "match " << i << " not found in map" << std::endl;
      continue;
    }

    int gpu_left_idx = iter_left->second;
    int gpu_right_idx = iter_right->second;

    /*
    std::cout << " left: " << cpu_left_idx << " -> " << gpu_left_idx << std::endl;
    std::cout << " right: " << cpu_right_idx << " -> " << gpu_right_idx << std::endl;
    */

    bool match_found = false;
    for (size_t j = 0; j < vgpu.getMatchesSize(); j++) {
      if (vgpu.getMatch(j).queryIdx == gpu_left_idx) {
        //std::cout << "query of " << i << " --> " << j << std::endl;
        match_found = true;
        if (vgpu.getMatch(j).trainIdx != gpu_right_idx) {
          //std::cerr << "match has wrong pair" << std::endl;
          errors++;
        }
        break;
      }
    }

    if (!match_found) {
      //std::cerr << "keypoint was not found in gpu view" << std::endl;
      errors++;
      continue;
    }
  }

  return errors;
}


bool time_feature_detection(double hessian, int octaves, int layers, bool extended, bool upright,
                            Frame *left, Frame *right)
{
  size_t n_runs = 10;

  std::cout << "=== CPU ===" << std::endl;

  std::vector<cv::Point2f> ckp_left;
  std::vector<cv::Point2f> ckp_right;
  ViewCombination vcpu = time_feature_detection("CPU", hessian, octaves, layers, extended, upright,
                                                left, right, ckp_left, ckp_right, n_runs);
  std::cout << "=== GPU ===" << std::endl;
  std::vector<cv::Point2f> gkp_left;
  std::vector<cv::Point2f> gkp_right;
  ViewCombination vgpu = time_feature_detection("GPU", hessian, octaves, layers, extended, upright,
                                                left, right, gkp_left, gkp_right, n_runs);

  std::cout << compare_points(ckp_left, gkp_left) << "/" << ckp_left.size()
            << " of left keypoints differ" << std::endl;
  std::cout << compare_points(ckp_right, gkp_right) << "/" << ckp_right.size()
            << " of right keypoints differ" << std::endl;

  std::cout << compare_matches(vcpu, vgpu)
              << "/" << vcpu.getMatchesSize() << " of matches differ" << std::endl;

  return true;
}

void test_feature_matching(Frame *left, Frame *right)
{
  int test = 3;
  bool color = false;

  double hessian = 500;
  double octaves = 5;
  int layers = 5;
  bool extended = true;
  bool upright = false;
  OpenCVFeatureDetector ocvfeat;
  ocvfeat.createDetector<cv::xfeatures2d::SURF>(hessian, octaves, layers, extended, upright);
  ocvfeat.createExtractor<cv::xfeatures2d::SURF>(hessian, octaves, layers, extended, upright);
  ocvfeat.createMatcher<cv::BFMatcher>(cv::NORM_L2, true);

  GPUSurf gpufeat(hessian, octaves, layers, extended, upright, cv::NORM_L2);

  OpticalFlowFeatures of;

  switch (test) {
    case 1:
      test_feature_matching_all(left, right);
      break;
    case 2:
      test_feature_matching_interactive(left, right);
      break;
    case 3:
    {
      // KEYPOINTWISE CPU
      ocvfeat.findFeatures(left);
      ocvfeat.findFeatures(right);
      ViewCombination view = ocvfeat.matchFrames(left, right);
      if (color) {
        test_show_features_colored(view, view.getMatchesSize());
      } else {
        test_show_features_iteratively(view);
      }
      break;
    }
    case 4:
    {
      // KEYPOINTWISE GPU
      gpufeat.findFeatures(left);
      gpufeat.findFeatures(right);
      ViewCombination view = gpufeat.matchFrames(left, right);
      if (color) {
        test_show_features_colored(view, view.getMatchesSize());
      } else {
        test_show_features_iteratively(view);
      }
      break;
    }
    case 5:
    {
      // KEYPOINTWISE OPTICAL FLOW
      ViewCombination view = of.matchFrames(left, right);
      if (color) {
        test_show_features_colored(view, view.getMatchesSize());
      } else {
        test_show_features_iteratively(view);
      }
      break;
    }
    case 6:
    {
      // TEST TIMES AND ACCURACY
      time_feature_detection(hessian, octaves, layers, extended, upright, left, right);
    }
  }
}

bool is_equal(cv::Vec4d const &lhs, cv::Vec4d const &rhs, double const epsilon)
{
  return (   (std::abs(lhs(0)/lhs(3)- rhs(0)/rhs(3)) < epsilon)
          && (std::abs(lhs(1)/lhs(3)- rhs(1)/rhs(3)) < epsilon)
          && (std::abs(lhs(2)/lhs(3)- rhs(2)/rhs(3)) < epsilon));
}

bool is_equal(double const &lhs, double const &rhs, double const epsilon)
{
  return ((std::abs(lhs- rhs) < epsilon));
}

struct GPUTriangulationTimes {
  double cpu_create;
  double cpu_process;
  double gpu_create;
  double gpu_process;
};

template <typename T>
bool compareTriangulationMethods(Camera const &cam, ViewCombination const &view,
                                 GPUTriangulationTimes &times, size_t const runs,
                                 bool use_zero_copy = false, bool print_errors = true)
{
  StopWatch sw;
  std::cout << " === Comparing CPU and GPU Iterative Linear LS using "
            << view.getMatchesSize() << " keypoints === " << std::endl
            << " --- Zero Copy " << (use_zero_copy ? "enabled" : "disabled")
            << " ---" << std::endl;

  sw.start();
  IterativeLinearLS<T> ref;
  sw.stop();
  times.cpu_create = sw.getDurationMs();

  sw.start();
  GPUIterativeLinearLS gpu(view.getMatchesSize(), use_zero_copy);
  sw.stop();
  times.gpu_create = sw.getDurationMs();

  std::cout << "creating CPU took " << times.cpu_create << "ms" << std::endl;
  std::cout << "creating GPU took " << times.gpu_create << "ms" << std::endl;

  double cpu_time = 0;
  double gpu_time = 0;

  for (size_t r = 0; r < runs; r++) {
    PointCloud ptsref;
    PointCloud ptsgpu;

    if (runs == 1) {
      std::cout << " === CPU triangulation === " << std::endl;
    }
    sw.start();
    ref.triangulate(cam, view, ptsref);
    sw.stop();
    cpu_time += sw.getDurationMs();

    if (runs == 1) {
      std::cout << " === GPU triangulation === " << std::endl;
    }
    sw.start();
    gpu.triangulate(cam, view, ptsgpu);
    sw.stop();
    gpu_time += sw.getDurationMs();

    size_t errors = 0;
    for (size_t i = 0; i < view.getMatchesSize(); i++) {
      if (!is_equal(ptsref.getPoint(i), ptsgpu.getPoint(i), 10e-9)
          || !is_equal(ptsref.getError(i), ptsgpu.getError(i), 10e-9)) {
        if (print_errors) {
          std::cout << ptsref.getPoint(i) << " --> " << ptsgpu.getPoint(i) << std::endl;
          std::cout << ptsref.getError(i) << " --> " << ptsgpu.getError(i) << std::endl;
        }
        errors++;
      }
    }

    if (print_errors) {
      std::cout << errors << "/" << view.getMatchesSize() << " points differ" << std::endl;
    }
    if (errors != 0) {
      return false;
    }
  }

  cpu_time /= runs;
  gpu_time /= runs;

  std::cout << "CPU took " << cpu_time << "ms" << std::endl;
  std::cout << "GPU took " << gpu_time << "ms" << std::endl;

  times.cpu_process = cpu_time;
  times.gpu_process = gpu_time;

  return true;
}

void testCublas()
{
  std::cout << "TESTING CUBLAS" << std::endl;

  /*
  cv::Mat_<float> cvA = (cv::Mat_<float>(4, 3) <<  1,  2,  3,
                                                   4,  5,  6,
                                                   7,  8,  9,
                                                  10, 11, 12);
                                                  */
  cv::Mat_<float> cvA = (cv::Mat_<float>(4, 3) <<  1,  0,  0,
                                                   0,  1,  0,
                                                   0,  0,  1,
                                                   0,  0,  0);
  cv::Mat_<float> cvB = (cv::Mat_<float>(4, 1) <<  1,
                                                   2,
                                                   3,
                                                   4);
  cv::Mat cvX;
  cv::solve(cvA, cvB, cvX, cv::DECOMP_QR);
  std::cout << "OpenCV: X = [" << cvX.at<float>(0) << ", "
                               << cvX.at<float>(1) << ", "
                               << cvX.at<float>(2) << "]" << std::endl;

  float **gpuA_arr = NULL;
  float *gpuA0[1];
  float **gpuB_arr = NULL;
  float *gpuB0[1];
  cudaMalloc(&gpuA_arr, sizeof(float *) * 1);
  cudaMalloc(&gpuA0[0], sizeof(float) * 12);
  cudaMemcpy(gpuA_arr, gpuA0, sizeof(float *), cudaMemcpyHostToDevice);
  cudaMalloc(&gpuB_arr, sizeof(float *) * 1);
  cudaMalloc(&gpuB0[0], sizeof(float) * 4);
  cudaMemcpy(gpuB_arr, gpuB0, sizeof(float *), cudaMemcpyHostToDevice);

  /*
  float gpuA[] = {1,  4,  7, 10,
                  2,  5,  8, 11,
                  3,  6,  9, 12 };
  */
  float gpuA[] = {1,  0,  0,  0,
                  0,  1,  0,  0,
                  0,  0,  1,  0 };
  float gpuB[] = {1, 2, 3, 4};
  cudaMemcpy(gpuA0[0], gpuA, sizeof(float) * 12, cudaMemcpyHostToDevice);
  cudaMemcpy(gpuB0[0], gpuB, sizeof(float) * 4, cudaMemcpyHostToDevice);

  int info;
  cublasHandle_t handle;
  if (cublasCreate_v2(&handle) != CUBLAS_STATUS_SUCCESS) {
    std::cout << "Could not create CUBLAS handle" << std::endl;
  }
  if (cublasSgelsBatched(handle, CUBLAS_OP_N,
                         4, 3, 1, gpuA_arr, 4, gpuB_arr, 4,
                         &info, NULL, 1) != CUBLAS_STATUS_SUCCESS) {
    std::cout << "Least Squares Solver: info=" << info << std::endl;
    return;
  }

  cudaMemcpy(gpuB0, gpuB_arr, sizeof(float *), cudaMemcpyDeviceToHost);
  cudaMemcpy(gpuB, gpuB0[0], sizeof(float) * 4, cudaMemcpyDeviceToHost);

  std::cout << "CUBLAS: X = [" << gpuB[0] << ", "
                               << gpuB[1] << ", "
                               << gpuB[2] << ", "
                               << gpuB[3] << "]" << std::endl;
  cublasDestroy(handle);
}

template <typename T>
void testGPUTriangulationSpeed(Camera const & cam, ViewCombination const &view)
{
#ifndef USE_CONSTANT_MEMORY
  FileWriter writer("gpu_triangulation_times.m");
#else
  FileWriter writer("gpu_triangulation_times_const.m");
#endif
  cv::Mat P_0, P_1;
  PointCloud pts;
  IterativeLinearLS<double> tria;
  OpenCVPoseEstimation pose(&tria);
  pose.estimatePose(cam, view, P_0, P_1, pts);

  view.getImageLeft()->setProjectionMatrix(P_0);
  view.getImageRight()->setProjectionMatrix(P_1);

  bool print_errors = false;
  size_t n_runs = 10;
  std::vector<size_t> n_points = { 1, 10, 20, 30, 40, 50, 60, 70, 80, 90,
                                   100, 200, 300, 400, 500, 600, 700, 800, 900, 1000,
                                   1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000,
                                   3000, 4000, 5000 };
  std::vector<GPUTriangulationTimes> times_copied(n_points.size());
  std::vector<GPUTriangulationTimes> times_mapped(n_points.size());

  for (size_t run = 0; run < n_points.size(); run++) {

    // build view with less points
    std::vector<cv::DMatch> matches;
    {
      for (size_t i = 0; i < n_points[run]; i++) {
        //matches.emplace_back(view.getMatch(i % view.getMatchesSize()));
        matches.emplace_back(view.getMatch(0));
      }
    }
    ViewCombination v2(view.getImageLeft(), view.getImageRight());
    v2.setMatches(matches);

    if (!compareTriangulationMethods<T>(cam, v2, times_copied[run], n_runs, false, print_errors)
        || !compareTriangulationMethods<T>(cam, v2, times_mapped[run], n_runs, true, print_errors)) {
      std::cerr << "ERRORS while triangulating" << std::endl;
      break;
    }
  }

  writer.writeNKeypoints(n_points);
#ifndef USE_CONSTANT_MEMORY
  writer.writeTimes("copied", times_copied);
  writer.writeTimes("mapped", times_mapped);
#else
  writer.writeTimes("const_copied", times_copied);
  writer.writeTimes("const_mapped", times_mapped);
#endif
}

template <typename T>
void testGPUTriangulationCorrectness(Camera const & cam, ViewCombination &view,
                                     bool use_zero_copy = false, double epsilon = 1e-4)
{
  bool print_errors = true;

  std::cout << " === Comparing CPU and GPU Iterative Linear LS using === " <<  std::endl;

  IterativeLinearLS<T> ref;
  GPUIterativeLinearLS gpu(view.getMatchesSize(), use_zero_copy);

  /*
  // build view with less points
  std::vector<cv::DMatch> matches;
  matches.emplace_back(view.getMatch(80));
  view.setMatches(matches);
  */

  PointCloud ptsref;
  PointCloud ptsgpu;

  std::cout << " === CPU triangulation === " << std::endl;
  ref.triangulate(cam, view, ptsref);

  std::cout << " === GPU triangulation === " << std::endl;
  gpu.triangulate(cam, view, ptsgpu);

  size_t errors = 0;
  T x_difference = 0;
  T y_difference = 0;
  T z_difference = 0;
  T w_difference = 0;
  T repr_difference = 0;
  for (size_t i = 0; i < view.getMatchesSize(); i++) {
    if (!is_equal(ptsref.getPoint(i), ptsgpu.getPoint(i), epsilon)
        || !is_equal(ptsref.getError(i), ptsgpu.getError(i), epsilon)) {
      if (print_errors) {
        std::cout << ptsref.getPoint(i) << " --> " << ptsgpu.getPoint(i) << std::endl;
        std::cout << ptsref.getError(i) << " --> " << ptsgpu.getError(i) << std::endl;
      }
      errors++;

      x_difference += std::abs(ptsref.getPoint(i)(0) - ptsgpu.getPoint(i)(0));
      y_difference += std::abs(ptsref.getPoint(i)(1) - ptsgpu.getPoint(i)(1));
      z_difference += std::abs(ptsref.getPoint(i)(2) - ptsgpu.getPoint(i)(2));
      w_difference += std::abs(ptsref.getPoint(i)(3) - ptsgpu.getPoint(i)(3));
      repr_difference += std::abs(ptsref.getError(i) - ptsgpu.getError(i));
    }
  }
  x_difference /= errors;
  y_difference /= errors;
  z_difference /= errors;
  w_difference /= errors;
  repr_difference /= errors;

  std::cout << errors << "/" << view.getMatchesSize() << " points differ" << std::endl;
  std::cout << " avg x error = " << x_difference << std::endl;
  std::cout << " avg y error = " << y_difference << std::endl;
  std::cout << " avg z error = " << z_difference << std::endl;
  std::cout << " avg w error = " << w_difference << std::endl;
  std::cout << " avg error error = " << repr_difference << std::endl;

}

void testGPUTriangulation(Camera const &cam, Frame *left, Frame *right)
{
  int test = 2;
  bool use_zero_copy = true;
  double epsilon = 10e-4;
  using type = double;

  RubikManualFeatures rub;
  ViewCombination view = rub.matchFrames(left, right);

  if (test == 2) {
    OpenCVFeatureDetector ocvfeat;
    ocvfeat.createExtractor<cv::xfeatures2d::SURF>(500, 5, 5, true, true);
    ocvfeat.createDetector<cv::xfeatures2d::SURF>(500, 5, 5, true, true);
    ocvfeat.createMatcher<cv::BFMatcher>(cv::NORM_L2, true);
    ocvfeat.findFeatures(left);
    ocvfeat.findFeatures(right);
    view = ocvfeat.matchFrames(left, right);
  }

  cv::Mat P_0, P_1;
  PointCloud pts;
  IterativeLinearLS<double> tria;
  OpenCVPoseEstimation pose(&tria);
  pose.estimatePose(cam, view, P_0, P_1, pts);

  view.getImageLeft()->setProjectionMatrix(P_0);
  view.getImageRight()->setProjectionMatrix(P_1);

  if (test == 1) {
    testGPUTriangulationSpeed<type>(cam, view);
  } else if (test == 2) {
    testGPUTriangulationCorrectness<type>(cam, view, use_zero_copy, epsilon);
  }
}
