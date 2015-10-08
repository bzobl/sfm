#include "livestream.h"

#include <cassert>
#include <iostream>

#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include "StopWatch.h"

LiveStream::LiveStream(int camNum) : LiveStream(camNum, -1, -1)
{
}

LiveStream::LiveStream(int camNum, int width, int height)
{
  if (!openCamera(camNum, width, height)) {
    return;
  }

  mOverlay = cv::Mat::zeros(mStreamHeight, mStreamWidth, CV_8UC3);
  resetOverlay();
  getCurrentFrame();
}

LiveStream::~LiveStream()
{
  stopPollingThread();
  if (mCamera.isOpened()) {
    mCamera.release();
  }
}

#define FOURCC(c1, c2, c3, c4) (((c1) & 255) + (((c2) & 255) << 8) + (((c3) & 255) << 16) + (((c4) & 255) << 24))

bool LiveStream::openCamera(int num, int width, int height)
{
  assert(num >= 0);
  mCamera.open(num);

  if (!mCamera.isOpened()) {
    std::cerr << "could not open camera" << std::endl;
    return false;
  }

  double codec = FOURCC('M', 'J', 'P', 'G');
  if (!mCamera.set(cv::CAP_PROP_FOURCC, codec))
  //if (!mCamera.set(cv::CAP_PROP_FOURCC, FOURCC('M', 'J', 'P', 'G')))
  {
    char *fourcc = (char *) &codec;
    std::cerr << "could not set codec " << fourcc[0] << fourcc[1] << fourcc[2] << fourcc[3] << std::endl;
  }

  if ((width != -1) && (height != -1)) {
    // both calls will return false
    mCamera.set(cv::CAP_PROP_FRAME_WIDTH, width);
    mCamera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  }

  mStreamWidth = mCamera.get(cv::CAP_PROP_FRAME_WIDTH);
  mStreamHeight = mCamera.get(cv::CAP_PROP_FRAME_HEIGHT);

  if (   ((width != -1) && (mStreamWidth != width))
      || ((height != -1) && (mStreamHeight != height))) {
    std::cerr << "could not set resolution " << width << "x" << height << std::endl;
    mCamera.release();
    return false;
  }

  std::cout << "initialized camera " << num << " with "
            << mStreamWidth << "x" << mStreamHeight << std::endl;
  return true;
}

void LiveStream::getCurrentFrame()
{
  /*
  cv::Mat yuv;
  mCamera.read(yuv);
  cv::cvtColor(yuv, mCurrentFrame, cv::COLOR_YUV2BGR);
  */
  /*
  cv::Mat jpg;
  mCamera.read(jpg);
  mCurrentFrame = cv::imdecode(jpg, 1);
  */
  mCamera.read(mCurrentFrame);
}

bool LiveStream::isOpened() const
{
  return mCamera.isOpened();
}

int LiveStream::width() const
{
  return mStreamWidth;
}

int LiveStream::height() const
{
  return mStreamHeight;
}

void LiveStream::getFrame(cv::Mat &frame)
{
  //std::unique_lock<std::mutex> l(mFrameMutex);

  mCurrentFrame.copyTo(frame);
}

void LiveStream::nextFrame(cv::Mat &frame)
{
  //std::unique_lock<std::mutex> l(mFrameMutex);

  getCurrentFrame();
  mCurrentFrame.copyTo(frame);
}

void LiveStream::startPollingThread(int fps)
{
  mThreadExit = false;
  // thread to query camera stream in 30 fps frequency
  // necessary, since reading from the camera does not return the latest
  // but the next image
  mPollingThread = new std::thread([this, fps](void)
                        {
                          double target_delay_ms = 1 / fps * 1000;
                          StopWatch timer;
                          while (!this->mThreadExit) {
                            timer.start();
                            this->frameTick();
                            timer.stop();
                            double delay = target_delay_ms - timer.getDurationMs();
                            if (delay > 0) {
                              std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(delay));
                            }
                          }
                        });
}

void LiveStream::stopPollingThread()
{
  if (mPollingThread != nullptr) {
    mThreadExit = true;

    mPollingThread->join();

    delete mPollingThread;
    mPollingThread = nullptr;
  }
}

void LiveStream::frameTick()
{
  //std::unique_lock<std::mutex> l(mFrameMutex);

  getCurrentFrame();
}

std::recursive_mutex &LiveStream::getOverlayMutex()
{
  return mOverlayMutex;
}

void LiveStream::resetOverlay()
{
  std::unique_lock<std::recursive_mutex> l(mOverlayMutex);
  mOverlayAlpha = cv::Mat::zeros(mStreamHeight, mStreamWidth, CV_8UC1);
  mOverlayTTL = DEFAULT_TTL;
}

void LiveStream::addImageToOverlay(AlphaImage const &image, int width, int x, int y)
{
  cv::Rect roi(x, y, image.width(width), image.height(width));
  image.write_scaled(mOverlay, mOverlayAlpha, roi);
}

void LiveStream::applyOverlay(cv::Mat &image)
{
  assert(image.cols == mStreamWidth);
  assert(image.rows == mStreamHeight);

  std::unique_lock<std::recursive_mutex> l(mOverlayMutex);

  for (int w = 0; w < mStreamWidth; w++) {
    for (int h = 0; h < mStreamHeight; h++) {
      if (mOverlayAlpha.at<uchar>(h, w) > 0) {
        image.at<cv::Vec3b>(h, w) = mOverlay.at<cv::Vec3b>(h, w);
      }
    }
  }

  mOverlayTTL--;
  if (mOverlayTTL <= 0) {
    resetOverlay();
  }
}
