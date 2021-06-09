// #pragma once
#include <stdint.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <unordered_set>
// #include <Eigen3/Eigen/Dense>
using namespace Eigen;
using namespace cv;
using namespace std;

struct VectorHash {
  size_t operator()(const std::vector<int>& v) const {
    std::hash<int> hasher;
    size_t seed = 0;
    for (int i : v) {
      seed ^= hasher(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};
class CoarseDM {
 public:
  CoarseDM();
  // Eigen::Matrix<int, 1681, 169> dictionaryMatrix(int pixelX, int pixelY,
  // cv::Mat imgTarget, cv::Mat imgRef);

  Eigen::VectorXf getGamma(int x, int y, const cv::Mat& img);
  Eigen::Matrix<float, Dynamic, Dynamic> dictionaryMatrix(
      int pixelX, int pixelY, const cv::Mat& imgRef);
  int getTargetErrorOMP(const Matrix<float, Dynamic, Dynamic>& dictA,
                        const VectorXf& targetY, VectorXf& xHatOut,
                        VectorXf& errorOut);
  std::vector<std::vector<int>> getSamplePoints(const cv::Mat& img);
  int interpolateOMPimage(
      const cv::Mat& img, cv::Mat& out, int x, int y,
      const std::unordered_set<std::vector<int>, VectorHash>&
          sample_points_set);
  void compareDM(cv::Mat& forwardDM, const cv::Mat& backwardDM);
  cv::Mat scaleImg(cv::Mat& img);
  int hsfc_last2bits(int x);
  Point hsfc_indexToXY(int hindex, int N);

 private:
  int rangeMap(int val, int in_start, int in_end, int out_start, int out_end);
  int flipRange(int in, int min, int max);
};
