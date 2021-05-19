// #pragma once
#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdint.h>
// #include <Eigen3/Eigen/Dense>
using namespace Eigen;
using namespace cv;
using namespace std;

class CoarseDM{
public:
  CoarseDM();
  // Eigen::Matrix<int, 1681, 169> dictionaryMatrix(int pixelX, int pixelY, cv::Mat imgTarget, cv::Mat imgRef);

  Eigen::VectorXf getGamma(int x, int y, const cv::Mat& img);
  Eigen::Matrix<float, Dynamic, Dynamic> dictionaryMatrix(int pixelX, int pixelY, const cv::Mat& imgRef);
  int getTargetErrorOMP(const Matrix<float, Dynamic, Dynamic>& dictA, const VectorXf& targetY, VectorXf& xHatOut, VectorXf& errorOut);
  std::vector<std::vector<int>> getSamplePoints(const cv::Mat& img);
  // void interpolateOMPimage(const cv::Mat& img, cv::Mat& out, int x, int y);
  int interpolateOMPimage(const cv::Mat& img, cv::Mat& out, int x, int y);
private:


};
