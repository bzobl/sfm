#ifndef FILE_WRITER_H_INCLUDED
#define FILE_WRITER_H_INCLUDED

#include <fstream>
#include <opencv2/core.hpp>

class FileWriter {

private:
  std::ofstream mFile;

public:
  FileWriter(std::string name) : mFile(name)
  {
  }

  virtual ~FileWriter()
  {
    mFile.close();
  }

  template <typename TPoint>
  void writePoints(std::string name, std::vector<TPoint> const &pts)
  {
    mFile << name << " = [ ";
    
    for (auto pt : pts) {
      mFile << pt << "' ..." << std::endl;
    }

    mFile << "];" << std::endl << std::endl;
  }

  void writeNKeypoints(std::vector<size_t> n_points)
  {
    mFile << "n_keypoints = [ ";

    for (size_t i = 0; i < n_points.size(); i++) {
      mFile << n_points[i] << " ";
    }

    mFile << "];" << std::endl << std::endl;
  }

  template <typename T>
  void writeTimes(std::string postfix, std::vector<T> const &times)
  {
    mFile << "time_cpu_create_" << postfix << " = [ ";
    for (size_t i = 0; i < times.size(); i++) {
      mFile << times[i].cpu_create << ", ";
    }
    mFile << "];" << std::endl << std::endl;

    mFile << "time_cpu_process_" << postfix << " = [ ";
    for (size_t i = 0; i < times.size(); i++) {
      mFile << times[i].cpu_process << ", ";
    }
    mFile << "];" << std::endl << std::endl;

    mFile << "time_gpu_create_" << postfix << " = [ ";
    for (size_t i = 0; i < times.size(); i++) {
      mFile << times[i].gpu_create << ", ";
    }
    mFile << "];" << std::endl << std::endl;

    mFile << "time_gpu_process_" << postfix << " = [ ";
    for (size_t i = 0; i < times.size(); i++) {
      mFile << times[i].gpu_process << ", ";
    }
    mFile << "];" << std::endl << std::endl;
  }
};

#endif
