// #pragma once
#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>
// #include <Eigen3/Eigen/Dense>
using namespace Eigen;
using namespace cv;

class CoarseDM{
public:
  CoarseDM();
  // Eigen::Matrix<int, 1681, 169> dictionaryMatrix(int pixelX, int pixelY, cv::Mat imgTarget, cv::Mat imgRef);

  Eigen::VectorXf getGamma(int x, int y, cv::Mat img);
  Eigen::Matrix<float, Dynamic, Dynamic> dictionaryMatrix(int pixelX, int pixelY, cv::Mat imgTarget, cv::Mat imgRef);
  // Eigen::Matrix<int, 2, 169> getTargetErrorOMP(Matrix<int, 1681, 169> dictA, VectorXi targetX);
  Eigen::Matrix<float, Dynamic, Dynamic> getTargetErrorOMP(const Matrix<float, Dynamic, Dynamic>& dictA, const VectorXf& targetX);
private:

};
