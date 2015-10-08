#include "PointCloud.h"

#include <algorithm>
#include <chrono>
#include <thread>

#include <pcl/visualization/pcl_visualizer.h>

void PointCloud::addPoint(cv::Vec4d point, double error,
                          std::pair<Frame const *, size_t> left_imgpt,
                          std::pair<Frame const *, size_t> right_imgpt)
{
  mPoints.push_back(point);
  mReprojectionErrors.push_back(error);

  std::pair<size_t, size_t> left_2d_3d = std::make_pair(left_imgpt.second, mPoints.size() - 1);
  std::pair<size_t, size_t> right_2d_3d = std::make_pair(right_imgpt.second, mPoints.size() - 1);
  mCorresponding2D[left_imgpt.first].push_back(left_2d_3d);
  mCorresponding2D[right_imgpt.first].push_back(right_2d_3d);
}

cv::Vec4d const &PointCloud::getPoint(int i) const
{
  return mPoints.at(i);
}

double const &PointCloud::getError(int i) const
{
  return mReprojectionErrors.at(i);
}

int PointCloud::findCorresponding3D(Frame const *frame, size_t const imgpt_idx) const
{
  auto iter = mCorresponding2D.find(frame);
  if (iter == mCorresponding2D.end()) {
    return -1;
  }

  std::vector<std::pair<size_t, size_t>> const &corr_2d_3d = iter->second;
  for (size_t i = 0; i < corr_2d_3d.size(); i++) {
    if (corr_2d_3d[i].first == imgpt_idx) {
      return corr_2d_3d[i].second;
    }
  }
  return -1;
}

double PointCloud::getAverageError() const
{
  double sum = std::accumulate(mReprojectionErrors.begin(), mReprojectionErrors.end(), 0.0);
  return sum / mReprojectionErrors.size();
}

cv::Point3d PointCloud::getPoint3D(size_t i) const
{
  return cv::Point3d(mPoints[i](0)/mPoints[i](3), mPoints[i](1)/mPoints[i](3), mPoints[i](2)/mPoints[i](3));
}

std::vector<cv::Point3d> PointCloud::getPoints3D() const
{
  std::vector<cv::Point3d> pts;
  for (auto &p : mPoints) {
    pts.emplace_back(p(0)/p(3), p(1)/p(3), p(2)/p(3));
  }
  return pts;
}

std::vector<cv::Vec4d> PointCloud::getPoints() const
{
  return mPoints;
}

size_t PointCloud::size() const
{
  return mPoints.size();
}

void PointCloud::clearPoints()
{
  mPoints.clear();
  mReprojectionErrors.clear();
}

bool PointCloud::isViewable(size_t i) const
{
  return (!std::isnan(mPoints[i](0)) &&
          !std::isnan(mPoints[i](1)) &&
          !std::isnan(mPoints[i](2)) &&
          (mPoints[i](3) > 0) &&
          (mReprojectionErrors[i] <= REPROJECTION_ERROR_THRESHOLD));
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud::createPclPointCloud()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  size_t skipped_points = 0;
  cv::Vec3b rgbv(0, 0, 0);
  uint32_t rgb = ((uint32_t)rgbv(0) << 16 | (uint32_t)rgbv(1) << 8 | (uint32_t)rgbv(2));

  for (size_t i = 0; i < mPoints.size(); i++) {
    cv::Vec4d pt = mPoints[i];

    if (!isViewable(i)) {
      skipped_points++;
      continue;
    }

    pcl::PointXYZRGB pclp;
    pclp.x = pt(0) / pt(3);
    pclp.y = pt(1) / pt(3);
    pclp.z = pt(2) / pt(3);

    pclp.rgb = *reinterpret_cast<float *>(&rgb);

    pcl_cloud->push_back(pclp);
  }

  std::cerr << "skipped " << skipped_points << "/" << mPoints.size() << " points for visualization" << std::endl;

  return pcl_cloud;
}

void PointCloud::RunVisualization(std::string name)
{
  std::cout << "creating viewer" << std::endl;
  pcl::visualization::PCLVisualizer viewer(name);
  viewer.setBackgroundColor(255, 255, 255);

  std::cout << "populating pointcloud" << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud = createPclPointCloud();

  std::cout << "adding pointcloud" << std::endl;
  viewer.addPointCloud(pcl_cloud);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10);

  std::cout << "starting viewer" << std::endl;
  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(100));
  }
}
