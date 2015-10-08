#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <map>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "frame.h"
#include "IterativeLinearLS.h"
#include "GPUIterativeLinearLS.h"
#include "GPUSurf.h"
#include "livestream.h"
#include "OpenCVFeatureDetector.h"
#include "OpenCVPoseEstimation.h"
#include "options.h"
#include "RubikManualFeatures.h"
#include "ViewCombination.h"
#include "OpticalFlowFeatures.h"
#include "Camera.h"
#include "StopWatch.h"
#include "FileWriter.h"
#include "tests.h"

#include "cuda/triangulation.cuh"

#include <cuda_runtime.h>
#include <cublas_v2.h>

using namespace std;

static void usage(char const * const progname, Options const &o)
{
  cout << "usage:" << endl
       << progname << " [OPTIONS]" << endl
       << endl
       << "Options:" << endl << o.print_all() << endl << endl;
}

// returns processed arguments
int check_options(Options &opts, int const argc, char const * const *argv)
{
  int i = 1;
  while(i < argc) {
    std::string arg(argv[i]);

    if (arg[0] != '-') {
      break;
    }

    if (arg == "--help") {
      return -1;
    }

    if (!opts.process(arg, i, argv)) {
      return -1;
    }
  }

  return i;
}

inline IFeatureDetect *getFeatureDetector(string name)
{
  double hessian = 800;
  double octaves = 5;
  int layers = 10;
  bool extended = true;
  bool upright = false;

  if (name == "RUB") {
    return new RubikManualFeatures();
  } else if (name == "OF" ) {
    return new OpticalFlowFeatures();
  } else if (name == "GPU" ) {
    return new GPUSurf(hessian, octaves, layers, extended, upright, cv::NORM_L2);
  } else if (name == "OCV" ) {
    OpenCVFeatureDetector *ocvfeat = new OpenCVFeatureDetector();
    ocvfeat->createExtractor<cv::xfeatures2d::SURF>(hessian, octaves, layers, extended, upright);
    ocvfeat->createDetector<cv::xfeatures2d::SURF>(hessian, octaves, layers, extended, upright);
    ocvfeat->createMatcher<cv::BFMatcher>(cv::NORM_L2, true);

    return ocvfeat;
  }

  return nullptr;
}

inline ITriangulation *getTriangulation(string name)
{
  if (name == "GPU") {
    return new GPUIterativeLinearLS(100);
  } else if (name == "OCV") {
    return new IterativeLinearLS<double>();
  }

  return nullptr;
}

inline void export_keypoints(ViewCombination const &view)
{
  FileWriter f("../matlab/keypoints.m");
  f.writePoints("x_1", view.getMatchingPointsLeft());
  f.writePoints("x_2", view.getMatchingPointsRight());
}

inline void show_matches(ViewCombination const &view)
{
  for (size_t i = 0; i < view.getMatchesSize(); i++) {
    if (view.getMatchingPointLeft(i) == view.getMatchingPointRight(i)) {
      //cout << "images share the same keypoint " << view.getKeypointsLeft()[i].pt << endl;
    }
  }

  cv::imshow("Left", view.getImageLeft()->drawKeypoints(view.getMatchingPointsLeft()));
  cv::imshow("Right", view.getImageRight()->drawKeypoints(view.getMatchingPointsRight()));
}

void test_features(LiveStream &stream, bool isLive, cv::Mat &img_last, cv::Mat &img_now)
{
  while (isLive) {
    cv::imshow("Last", img_last);
    cv::imshow("Now", img_now);
    char key = cv::waitKey(30);

    if (key == 'n') {
      img_now.copyTo(img_last);
      stream.getFrame(img_now);
    }
    if (key == 'q') break;
  }
  Frame frame_last(img_last);
  Frame frame_now(img_now);
  test_feature_matching(&frame_now, &frame_last);
}

bool initial_pose_estimation(Camera const &cam, ViewCombination &view, IPoseEstimation *poseestim,
                             bool show_points, bool debug = false)
{
  cv::Mat P_0, P_1;
  PointCloud pts;

  if (!poseestim->estimatePose(cam, view, P_0, P_1, pts)) {
    return false;
  }

  if (debug) {
    std::cout << "Initial Pose estimation successful" << std::endl;
    std::cout << "P_0" << std::endl << P_0 << std::endl;
    std::cout << "P_1" << std::endl << P_1 << std::endl;
    std::cout << "Triangulated " << pts.size() << " 3d points" << std::endl;
  }

  if (show_points) {
    pts.RunVisualization("Initial pose estimation");
  }

  std::vector<std::pair<size_t, double>> minimalErrors;
  for (size_t i = 0; i < pts.size(); i++) {
    minimalErrors.emplace_back(i, pts.getError(i));
  }
  std::sort(minimalErrors.begin(), minimalErrors.end(),
            [](std::pair<size_t, double> const &lhs, std::pair<size_t, double> const &rhs)
            {
              return lhs.second < rhs.second;
            });

  std::vector<cv::Point2f> img_points_left;
  std::vector<cv::Point2f> img_points_right;
  std::vector<cv::Point3f> points_3d;
  for (size_t i = 0;
       (i < minimalErrors.size()) && (minimalErrors[i].second <= 50.0);
       i++) {
    size_t idx = minimalErrors[i].first;

    if (pts.getPoint(idx)(3) <= 0) continue;

    img_points_left.emplace_back(view.getMatchingPointLeft(idx));
    img_points_right.emplace_back(view.getMatchingPointRight(idx));
    cv::Vec4d const &pt = pts.getPoint(idx);
    points_3d.emplace_back(pt(0), pt(1), pt(2));
  }

  if (debug) {
    std::cout << "found " << points_3d.size() << " points elegible for PnP" << std::endl;
  }

  if (points_3d.size() <= 6) {
    return false;
  }

  try {
    if (debug) {
      std::cout << "PnP of left view" << std::endl;
    }
    poseestim->estimatePose(cam, *view.getImageLeft(), points_3d, img_points_left);

    if (debug) {
      std::cout << "PnP of right view" << std::endl;
    }
    poseestim->estimatePose(cam, *view.getImageRight(), points_3d, img_points_right);
  } catch (std::exception e) {
    std::cerr << "PnP after initial pose estimation failed: " << e.what();
    return false;
  }

  return true;
}

bool find_2D_3D_correspndences(Frame *now_frame, std::vector<Frame *> const &frames,
                               PointCloud const &pts,
                               IFeatureDetect *fdetect,
                               std::vector<cv::Point2f> &points_2d,
                               std::vector<cv::Point3f> &points_3d)
{
  const double ERROR_THRESHOLD = 10;
  const double DISTANCE_THRESHOLD = 0.3;

  points_2d.clear();
  points_3d.clear();

  for (Frame *last_frame : frames) {
    //Frame *last_frame = frames[frames.size() - 2];

    ViewCombination view = fdetect->matchFrames(last_frame, now_frame);

    for (cv::DMatch &m : view.getMatches()) {
      int idx_3d = pts.findCorresponding3D(last_frame, m.queryIdx);
      if (idx_3d != -1) {
        if (!pts.isViewable(idx_3d)) continue;
        if (pts.getError(idx_3d) > ERROR_THRESHOLD) continue;
        if (m.distance > DISTANCE_THRESHOLD) continue;
          points_2d.push_back(now_frame->getImagepoint(m.trainIdx));
          points_3d.push_back(pts.getPoint3D(idx_3d));
          /*
          std::cout << "found 2D-3D correspondence: " << points_2d.back() << " -->"
                                                      << points_3d.back()
                                                      << " error=" << pts.getError(idx_3d)
                                                      << std::endl;
                                                      */
      }
    }

    if (points_2d.size() > 10) {
      return true;
    }
  }

  return false;
}

bool wait_for_img(LiveStream &stream, cv::Mat const &last_img, cv::Mat &new_img, IFeatureDetect *fdetect)
{
  bool breakwhile = false;
  bool quit = false;
  Frame tmp_last(last_img);
  fdetect->findFeatures(&tmp_last);
  std::vector<cv::Point2f> kp_last;
  std::vector<cv::Point2f> kp_now;

  do {
    stream.getFrame(new_img);
    Frame tmp_now(new_img);
    cv::imshow("Last", tmp_last.drawKeypoints(kp_last));
    cv::imshow("Now", tmp_now.drawKeypoints(kp_now));
    char key = cv::waitKey(30);

    switch (key) {

      case 'c':
      {
        fdetect->findFeatures(&tmp_now);
        ViewCombination view = fdetect->matchFrames(&tmp_last, &tmp_now);
        kp_last = view.getMatchingPointsLeft();
        kp_now = view.getMatchingPointsRight();
        break;
      }
      case 'q':
        quit = true;
      case 'n':
        cv::destroyWindow("Last");
        cv::destroyWindow("Now");
        breakwhile = true;
        break;
    }
  } while(!breakwhile);

  return quit;
}

typedef struct {
  double feature_detection;
  double pose_estimation;
  double triangulation;
} SfMTimes;

std::ostream &operator<<(std::ostream &out, SfMTimes const &times)
{
  out << "Feature Detect took  " << times.feature_detection << " ms" << endl
      << "Pose Estimation took " << times.pose_estimation << " ms" << endl
      << "Triangulation took   " << times.triangulation << " ms" << endl
      << "SfM took             " << times.feature_detection +
                                  + times.pose_estimation +
                                  + times.triangulation;
  return out;
}

bool process_sfm(IFeatureDetect *fdetect, IPoseEstimation *poseestim, ITriangulation *triang,
                 Camera const &cam,
                 Frame *frame_last, Frame *frame_now, std::vector<Frame *> const &frames,
                 PointCloud &global_points, SfMTimes &times,
                 bool initial_pose_estimated, bool export_kp, bool show_matches, bool show_points)
{
  double const DISTANCE_THRESHOLD = 10;

  StopWatch fdsw;
  fdetect->findFeatures(frame_now);
  ViewCombination view = fdetect->matchFrames(frame_last, frame_now);

  if (export_kp) { export_keypoints(view); }
  if (show_matches) { test_show_features_colored(view, view.getMatchesSize(), true); }

  double average_distance = 0.0;
  for (size_t i = 0; i < view.getMatchesSize(); i++) {
    double distance = ITriangulation::norm(view.getMatchingPointLeft(i),
                                           view.getMatchingPointRight(i));
    average_distance += distance;
  }
  average_distance /= view.getMatchesSize();
  fdsw.stop();
  std::cout << "average_distance between " << view.getMatchesSize()
            << " matches: " << average_distance << std::endl;

  if (average_distance < DISTANCE_THRESHOLD) {
    std::cerr << "camera has not moved" << std::endl;
    return false;
  }

  StopWatch pesw;
  if (!initial_pose_estimated) {
    std::cout << "Estimating initial pose" << std::endl;
    if (!initial_pose_estimation(cam, view, poseestim, show_points)) {
      std::cerr << "Pose estimation failed" << std::endl;
      return false;
    }
  } else {
    std::vector<cv::Point2f> points_2d;
    std::vector<cv::Point3f> points_3d;

    std::cout << "finding 2D 3D correspondences" << std::endl;
    if (!find_2D_3D_correspndences(frame_now, frames, global_points,
                                   fdetect, points_2d, points_3d)) {
      std::cerr << "could not find enough correspondences" << std::endl;
      return false;
    }

    std::cout << "Starting PnP" << std::endl;
    if (!poseestim->estimatePose(cam, *frame_now, points_3d, points_2d)) {
      std::cerr << "PnP failed" << std::endl;
      return false;
    }
  }
  pesw.stop();

  std::cout << "Starting triangulation of " << view.getMatchesSize() << " keypoints" << std::endl;
  StopWatch trsw;
  if (!triang->triangulate(cam, view, global_points)) {
    std::cerr << "Triangulation failed" << std::endl;
    return false;
  }
  trsw.stop();
  /*
  OpticalFlowFeatures of;
  std::cout << "starting of" << std::endl;
  ViewCombination vof = of.matchFrames(frame_last, frame_now);
  std::cout << "matches:  " << vof.getMatchesSize() << std::endl
            << "kp left:  " << frame_last->getKeypointsSize() << std::endl
            << "kp right: " << frame_now->getKeypointsSize() << std::endl;
  std::cout << "starting triangulating of" << std::endl;
  triang->triangulate(cam, vof, 
                      view.getImageLeft()->getProjectionMatrix(),
                      view.getImageRight()->getProjectionMatrix(),
                      global_points);
  //of_pts.RunVisualization();
  */
  /*
  OpticalFlowFeatures of;
  std::cout << "starting of" << std::endl;
  Frame f1(img_last);
  Frame f2(img_now);
  ViewCombination vof = of.matchFrames(&f1, &f2);
  triang->triangulate(cam, vof, 
                      frame_last->getProjectionMatrix(),
                      frame_now->getProjectionMatrix(),
                      global_points);
                      */

  if (show_points) {
    global_points.RunVisualization("Global points in sfm loop");
  }

  times.feature_detection = fdsw.getDurationMs();
  times.pose_estimation = pesw.getDurationMs();
  times.triangulation = trsw.getDurationMs();
  return true;
}

int main(int argc, char **argv)
{
  Options opts;
  int nopts = check_options(opts, argc, argv);
  if (nopts == -1) {
    usage(argv[0], opts);
    return 1;
  }
  argc -= nopts;
  argv += nopts;

  cout << "Options:" << endl << opts << endl;

  LiveStream stream(opts.cam_num, opts.width, opts.height);
  if (opts.live_stream) {
    if (!stream.isOpened()) {
    std::cerr << "Could not open camera " << opts.cam_num.value
              << " with " << opts.width.value << " x " << opts.height.value
              << " pixel" << std::endl;
    return -1;
    } else {
      stream.startPollingThread(30);
    }
  }

  cv::Mat img_last, img_now;

  if (!opts.live_stream) {
    string img1_name = "../matlab/rubik/rubik01.jpg";
    string img2_name = "../matlab/rubik/rubik02.jpg";
    //string img1_name = "./test_imgs/room01.jpg";
    //string img2_name = "./test_imgs/room02.jpg";
    //string img1_name = "./test_imgs/boxes01.png";
    //string img2_name = "./test_imgs/boxes02.png";
    //string img1_name = "./test_imgs/Lenna.png";
    //string img2_name = "./test_imgs/Lenna02.png";
    img_last = cv::imread(img1_name);
    img_now = cv::imread(img2_name);
  } else {
    stream.getFrame(img_last);
    stream.getFrame(img_now);
  }

  if (opts.test_features) {
    test_features(stream, opts.live_stream, img_last, img_now);
    return 0;
  }

  if (opts.test_triangulation) {
    Frame frame_last(img_last);
    Frame frame_now(img_now);
    Camera cam(img_now.cols, img_now.rows);
    testGPUTriangulation(cam, &frame_last, &frame_now);
    return 0;
  }

  Camera cam(img_now.cols, img_now.rows);
  IFeatureDetect *fdetect = getFeatureDetector(opts.feat_detector);
  ITriangulation *triang = getTriangulation(opts.triangulation);
  IPoseEstimation *poseestim = new OpenCVPoseEstimation(triang);

  assert(fdetect != NULL);
  assert(triang != NULL);
  assert(poseestim != NULL);

  Frame *frame_last = new Frame(img_last);
  Frame *frame_now = frame_last;
  std::vector<Frame *> frames;
  frames.push_back(frame_now);
  if (opts.feat_detector.value == "RUB") {
    RubikManualFeatures *rub = dynamic_cast<RubikManualFeatures *>(fdetect);
    rub->preloadFrame(frame_now, 1);
  }
  fdetect->findFeatures(frame_now);

  size_t rubik_counter = 2;
  bool initial_pose_estimated = false;
  PointCloud global_points;

  SfMTimes times_initial = { 0 };
  std::vector<SfMTimes> times_average;
  size_t times_counter = 0;

  do {
    frame_last = frame_now;
    img_last = img_now;

    if (opts.live_stream) {
      /*
      stream.getFrame(img_now);
      cv::imshow("Live", img_now);
      char key = cv::waitKey(30);
      if (key == 'q') break;
      */
      if (wait_for_img(stream, img_last, img_now, fdetect)) {
        break;
      }
    } 

    frame_now = new Frame(img_now);
    frames.push_back(frame_now);

    if (opts.feat_detector.value == "RUB") {
      if (rubik_counter > 4) {
        std::cout << "enough  rubik iterations" << std::endl;
        break;
      }
      std::cout << "=== Processing RUBIK features #" << rubik_counter << " ===" << std::endl;
      RubikManualFeatures *rub = dynamic_cast<RubikManualFeatures *>(fdetect);
      rub->preloadFrame(frame_now, rubik_counter++);
    }

    times_average.emplace_back();
    if (process_sfm(fdetect, poseestim, triang, cam,
                    frame_last, frame_now, frames, global_points, times_average[times_counter],
                    initial_pose_estimated, opts.export_keypoints, opts.show_matches,
                    opts.show_points))
    {
      cout << "SfM took " << std::endl << times_average[times_counter] << endl;
      if (!initial_pose_estimated) {
        times_initial = times_average[times_counter];
        times_average.pop_back();
        initial_pose_estimated = true;
      } else {
        times_counter++;
      }
    }

  } while (opts.live_stream || (opts.feat_detector.value == "RUB"));

  SfMTimes avg_times;
  for_each(times_average.begin(), times_average.end(), [&avg_times](SfMTimes const &t)
           {
            avg_times.feature_detection += t.feature_detection;
            avg_times.pose_estimation += t.pose_estimation;
            avg_times.triangulation += t.triangulation;
           });
  avg_times.feature_detection /= times_average.size();
  avg_times.pose_estimation /= times_average.size();
  avg_times.triangulation /= times_average.size();

  cout << "------ SfM run " << times_counter + 1 << " times ------" << endl;
  cout << "initial " << std::endl << times_initial << endl;
  cout << "average " << std::endl << avg_times << endl;
  cout << "------------------------------------------- " << endl;

  std::cout << "Starting visualization" << std::endl;
  global_points.RunVisualization("Global points");

  delete fdetect;
  delete poseestim;
  delete triang;

  return 0;
}
