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
Eigen::VectorXf CoarseDM::getGamma(int x, int y, cv::Mat img) {
   Eigen::VectorXf result(169);
   int i = 0;
   int yOffset = (13-1) / 2;
   int xOffset = (13-1) / 2;
   for (int curY = y - yOffset; curY <= y + yOffset; curY++) {
      for (int curX = x - xOffset; curX <= x + xOffset; curX++) {
         result(i) = img.at<float>(curY, curX);
         // result(i) = img.at<uchar>(curX, curY);
         // cout << result(i) << "\n";
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
Eigen::Matrix<float, Dynamic, Dynamic> CoarseDM::dictionaryMatrix(int pixelX, int pixelY, cv::Mat imgTarget, cv::Mat imgRef) {
   // Scalar intensity = img.at<uchar>(Point(x, y));
   // Eigen::MatrixXi result(1682, 169);
   Eigen::MatrixXf result(169, 1681);
   result.fill(0);
   int i = 0;
   // 41 x 41 search space
   int yOffset = 20;
   int xOffset = 20;

   // cout << "start from center " << pixelX << ", " << pixelY << "\n";
   for (int curY = pixelY - yOffset; curY <= pixelY + yOffset; curY++) {
      for (int curX = pixelX - xOffset; curX <= pixelX + xOffset; curX++) {
         result.col(i) = getGamma(curX, curY, imgRef);
         // cout << "coords: " << curX << ", " << curY << "\n";
         // cout << "column: \n" << result.col(i) << "\n";
         i++;
      }
   }
   // cout << i << "\n";
   return result;
}

// Given dictionary matrix A, target vector y
// Outputs gamma vector x, error vector e (stacked into a 2 x 169 matrix)
// such that y - Ax = e, via Orthogonal Matching Pursuit
Eigen::Matrix<float, Dynamic, Dynamic> CoarseDM::getTargetErrorOMP(const Eigen::Matrix<float, Dynamic, Dynamic>& dictA, const Eigen::VectorXf& targetY) {
  // Setup
  Eigen::MatrixXf result(dictA.cols(), 2);
  result.fill(0);

  Eigen::MatrixXf xHat(dictA.cols(), 1);
  xHat.fill(0);

  // reshape targetY if needed: rename parameter to "oldY"
  // Eigen::MatrixXi targetY(1, 169);
  // // targetY.row(0) = oldY.col(0);
  // // Eigen::MatrixXi targetY(169, 1);
  // targetY = oldY;


  Eigen::VectorXf error(dictA.cols()); // make error vector, duplicate values in targetY
  error = targetY;
  cout << "targetY: \n" << targetY << "\n";
  cout << "end targetY \n";
  // cout << "error: \n" << error << "\n end error \n";

  float errorThresh = 0.1; // placeholder
  int maxIterations = 10;  // placeholder

  int iteration = 0;
  Eigen::VectorXi support(maxIterations); // init support vector
  support.fill(0);
  float errorNorm = error.norm();
  // cout << "error norm: " << errorNorm << "\n";
  while (errorNorm > errorThresh && iteration < maxIterations) {

    float maxValue = 0;
    int maxIndex;
    float curValue = 0;
    Eigen::VectorXf curGamma(dictA.rows());
    curGamma.fill(0);
    for (int j = 0; j < dictA.cols(); j++) {
      curGamma = dictA.col(j);
      curValue = abs((curGamma.transpose() * error).norm() / curGamma.norm());

      if (curValue > maxValue) {
        maxIndex = j;
        maxValue = curValue;
      }
    }

    cout << "max value: " << maxValue << "\n";
    cout << "support chose column: " << maxIndex << "\n";
    support(iteration) = maxIndex;
    iteration++;

    // update error, errorNorm, xHat

    Eigen::MatrixXf dictA_pseudo_inv = dictA.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXf xTemp = dictA_pseudo_inv * targetY;

    xHat.fill(0);
    for (int j = 0; j < iteration; j++) {
      int support_index = support(j);
      xHat(support(j)) = xTemp(support(j));
    }
    // cout << "xHat\n" << xHat << "end xHat\n";

    error = targetY - (dictA * xHat);

    float oldErrorNorm = errorNorm;
    errorNorm = error.norm();
    float normDelta = oldErrorNorm - errorNorm;
    // cout << "error delta (of norm): " << normDelta << "\n";

    cout << "error norm: " << errorNorm << "\n";
  }

  // result.col(0) = xHat.col(0);
  result.col(0) = xHat;
  result.col(1).head(dictA.rows()) = error;
  return result;
}

// int main (int argc, char *argv[]) {
//   return 0;
// }
