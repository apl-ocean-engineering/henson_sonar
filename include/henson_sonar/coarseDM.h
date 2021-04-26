// #pragma once
#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
// #include <Eigen3/Eigen/Dense>
using namespace Eigen;
using namespace cv;
using namespace std;

class CoarseDM{
public:
  CoarseDM();
  // Eigen::Matrix<int, 1681, 169> dictionaryMatrix(int pixelX, int pixelY, cv::Mat imgTarget, cv::Mat imgRef);

  Eigen::VectorXf getGamma(int x, int y, cv::Mat img);
  Eigen::Matrix<float, Dynamic, Dynamic> dictionaryMatrix(int pixelX, int pixelY, cv::Mat imgTarget, cv::Mat imgRef);
  // Eigen::Matrix<int, 2, 169> getTargetErrorOMP(Matrix<int, 1681, 169> dictA, VectorXi targetX);
  // Eigen::Matrix<float, Dynamic, Dynamic> getTargetErrorOMP(const Matrix<float, Dynamic, Dynamic>& dictA, const VectorXf& targetX);
  // void getTargetErrorOMP(Matrix<float, Dynamic, Dynamic>& dictA, VectorXf& targetY, VectorXf& xHatOut, VectorXf& errorOut);
  float getTargetErrorOMP(Matrix<float, Dynamic, Dynamic>& dictA, VectorXf& targetY);
  std::vector<std::vector<int>> getSamplePoints(cv::Mat img);
private:


};
