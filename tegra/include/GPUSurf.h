#ifndef GPU_SURF_H_INCLUDED
#define GPU_SURF_H_INCLUDED

#include "IFeatureDetect.h"
#include "GPUHelper.h"

#include <opencv2/xfeatures2d/cuda.hpp>
#include <cuda_runtime.h>

class StaticAllocator : public cv::cuda::GpuMat::Allocator {

  private:

    unsigned char *mData = nullptr;
    size_t mLen = 0;
    size_t mStep = 0;

    void allocate(int rows, int cols, size_t elemSize)
    {
      mLen = elemSize * cols * rows;
      if (rows > 1 && cols > 1)
      {
        GPUHelper::check(cudaMallocPitch(&mData, &mStep, elemSize * cols, rows));
      }
      else
      {
        //Single row or single column must be continuous
        GPUHelper::check(cudaMalloc(&mData, mLen));
        mStep = elemSize * cols;
      }
    }

    void free()
    {
      if (mData != nullptr) {
        GPUHelper::check(cudaFree(mData));
      }
      mData = nullptr;
      mLen = 0;
    }

  public:

  ~StaticAllocator()
  {
    this->free();
  }

  virtual bool allocate(cv::cuda::GpuMat *mat, int rows, int cols, size_t elemSize)
  {
    if ((mData == nullptr) || ((rows * cols * elemSize) > mLen)) {
      this->free();
      this->allocate(rows, cols, elemSize);
    }

    mat->data = mData;
    mat->step = mStep;
    mat->refcount = new int;
    return true;
  }

  virtual void free(cv::cuda::GpuMat *mat)
  {
    delete mat->refcount;
  }

};

class GPUSurf : public IFeatureDetect {

private:
  cv::cuda::SURF_CUDA mSurf;
  StaticAllocator mImageAllocator;
  StaticAllocator mKeypointsAllocator;
  StaticAllocator mMaskAllocator;

  int mNorm;

public:
  GPUSurf(double hessian = 500, int octaves = 5, int layers = 5,
          bool extended = true, bool upright = true,
          int norm = cv::NORM_L2, float ratio = 0.1);

  virtual bool findFeatures(Frame *frame);
  virtual ViewCombination matchFrames(Frame *left, Frame *right);
};

#endif
