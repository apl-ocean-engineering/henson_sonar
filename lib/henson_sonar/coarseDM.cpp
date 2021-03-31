// take in pixel coordinate, openCV target image, openCV reference image
// spit out target gamma vector (1 x 169), dictionary matrix of gamma vectors
//    in search area of reference image (1681 x 169)

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>
#include "henson_sonar/coarseDM.h"

using namespace Eigen;
using namespace cv;
using namespace std;

CoarseDM::CoarseDM(){

}

// Given image, coordinates
// Outputs gamma vector centered at given coordinates of given image
// Assumes 13 x 13 area, single channel grayscale image (type 8UC1)
Eigen::VectorXi CoarseDM::getGamma(int x, int y, cv::Mat img) {
   Eigen::VectorXi result(169);
   int i = 0;
   int yOffset = (13-1) / 2;
   int xOffset = (13-1) / 2;
   for (int curY = y - yOffset; curY < y + yOffset; curY++) {
      for (int curX = x - xOffset; curX < x + xOffset; curX++) {
         result(i) = img.at<uchar>(curY, curX);
         i++;
      }
   }
   return result;
}

// Takes target pixel coords, target image, reference image
// Outputs dictionary matrix A of reference image for used in OMP estimation
// First column of output = target gamma, rest of matrix = dictionary matrix of
// gamma vectors in reference image
// Assumes 13 x 13 area for gamma vectors,
//         41 x 41 search area in reference image

// note: might need Eigen:: at start of next line
// note: user must get target gamma via: getGamma(pixelX, pixelY, imgTarget);
// Eigen::Matrix<int, 169, 1681> dictionaryMatrix(int pixelX, int pixelY, cv::Mat imgTarget, cv::Mat imgRef) {
Eigen::Matrix<int, Dynamic, Dynamic> CoarseDM::dictionaryMatrix(int pixelX, int pixelY, cv::Mat imgTarget, cv::Mat imgRef) {
   // Scalar intensity = img.at<uchar>(Point(x, y));
   // Eigen::MatrixXi result(1682, 169);
   Eigen::MatrixXi result(169, 1681);
   int i = 0;
   // offsets: 41 x 41 total search, 13 x 13 area per gamma ->
   // center coord of top left gamma = (x-20+6, y-20+6) = (x-14, y-14)
   // (using offset from center)
   int yOffset = 14;
   int xOffset = 14;
   for (int curY = pixelY - yOffset; curY < pixelY + yOffset; curY++) {
      for (int curX = pixelX - xOffset; curX < pixelX + xOffset; curX++) {
         result.col(i) = getGamma(curX, curY, imgRef);
         i++;
      }
   }
   return result;
}

// Given dictionary matrix A, target vector y
// Outputs gamma vector x, error vector e (stacked into a 2 x 169 matrix)
// such that y - Ax = e, via Orthogonal Matching Pursuit
Eigen::Matrix<int, Dynamic, Dynamic> CoarseDM::getTargetErrorOMP(const Eigen::Matrix<int, Dynamic, Dynamic>& dictA, const Eigen::VectorXi& targetY) {
  // Setup
  Eigen::MatrixXi result(169, 2);
  result.fill(0);

  Eigen::MatrixXi xHat(1, 169);
  // Eigen::MatrixXi xHat(169, 1);
  xHat.fill(0);

  // reshape targetY if needed: rename parameter to "oldY"
  // Eigen::MatrixXi targetY(1, 169);
  // // targetY.row(0) = oldY.col(0);
  // // Eigen::MatrixXi targetY(169, 1);
  // targetY = oldY;

  Eigen::VectorXi error(169); // make error vector, duplicate values in targetY
  for (int i = 0; i < error.size(); i++) {
    error(i) = targetY(i);
  }


  float errorThresh = 0.1; // placeholder
  int maxIterations = 10;  // placeholder
  int iteration = 0;
  Eigen::VectorXi support(maxIterations); // init support vector
  support.fill(0);
  // Loop
  float errorNorm = error.cast<float>().norm();
  while (errorNorm > errorThresh && iteration < maxIterations) {

    auto maxValue = 0;
    int maxIndex;
    float curValue;
    Eigen::VectorXi curGamma(169);
    curGamma.fill(0);
    for (int j = 0; j < dictA.cols(); j++) {
      curGamma = dictA.col(j);
      float gammaNorm = curGamma.cast<float>().norm();
      // the norm of the matrix product between (curGamma transpose)
      // and the current error signal, divided by the
      // euclidean norm of curGamma
      // printf("multiply at line 98\n");
      curValue = abs((curGamma.transpose() * error) / gammaNorm);
      // printf("success line 98\n");

      if (curValue > maxValue) {
        maxIndex = j;
      }
    }

    support(iteration) = maxIndex;
    iteration++;
    // update xHat
    for (int j = 0; j < iteration; j++) {
      xHat(j) = support(j);
    }
    // update error, errorNorm
    printf("multiply at line 114\n");

    printf("xHat rows: %ld\n", xHat.rows());
    printf("xHat cols: %ld\n", xHat.cols());

    printf("dictA rows: %ld\n", dictA.rows());
    printf("dictA cols: %ld\n", dictA.cols());

    printf("targetY rows: %ld\n", targetY.rows());
    printf("targetY cols: %ld\n", targetY.cols());

    printf("error rows: %ld\n", error.rows());
    printf("error cols: %ld\n", error.cols());

    // GOAL:
    // error = targetY - (dictA * xHat);

    Eigen::MatrixXi test;
    test = dictA * xHat; // this currently fails

    // trying other ways of multiplying dictA * xHat
    // error = targetY - dictA.cwiseProduct(xHat);
    // for (int k = 0; k < dictA.cols(); k++) {
    //   printf("test\n");
    //   tempDict.col(k) = dictA.col(k) * xHat;
    //   printf("success\n");
    // }
    // error = targetY - tempDict.rowwise().sum();
    // MatrixXi test;
    // // test = xHat.cwiseProduct(dictA);
    // // // test = dictA.cwiseProduct(xHat);
    // test = xHat.asDiagonal() * dictA;
    // cout << test << "\n";
    // printf("test cols: %ld\n", test.cols());
    // rowSum = test.rowwise().sum();
    // printf("rowSum cols: %ld", rowSum.cols());
    // cout << rowSum << "\n";
    // cout << test << "\n";
    // error = targetY - dictA.cwiseProduct(xHat).sum();
    // printf("success line 114\n");
    // errorNorm = error.cast<float>().norm();
  }

  result.col(0) = xHat.col(0);
  result.col(1) = error;
  return result;
}

// int main (int argc, char *argv[]) {
//   return 0;
// }
