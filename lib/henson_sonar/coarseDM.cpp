// take in pixel coordinate, openCV target image, openCV reference image
// spit out target gamma vector (1 x 169), dictionary matrix of gamma vectors
//    in search area of reference image (1681 x 169)

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "henson_sonar/coarseDM.h"
#include "ros/ros.h"
#include <fstream>

#include <ros/console.h>

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
// void getTargetErrorOMP(const Matrix<float, Dynamic, Dynamic>& dictA, const VectorXf& targetY, VectorXf& xHatOut, VectorXf& errorOut) {
float CoarseDM::getTargetErrorOMP(const Matrix<float, Dynamic, Dynamic>& dictA, const VectorXf& targetY, VectorXf& xHatOut, VectorXf& errorOut) {
  // xHatOut.resize(dictA.cols());
  // errorOut.resize(dictA.rows());
  // xHatOut.fill(0);
  // errorOut.fill(0);

  // cout << dictA << "\n";

  ROS_INFO("PROCESSING NEW IMAGE");

  Eigen::VectorXf xHat(dictA.cols());
  xHat.fill(0);

  Eigen::VectorXf error(dictA.rows()); // make error vector, duplicate values in targetY
  error = targetY/targetY.norm();

  float errorThresh = 0.1; // placeholder
  int maxIterations = 100;  // placeholder

  int iteration = 0;

   std::vector<int> supports;
  float errorNorm = error.norm();
   while (true){
   float maxValue = 0;
   int maxIndex;
    for (int j=0; j<dictA.cols(); j++){
       Eigen::VectorXf Aj = dictA.col(j);
       float currValue = abs(Aj.transpose()*error); // curGamma.norm();
       currValue /= Aj.norm();
       if (currValue > maxValue) {
          maxIndex = j;
          maxValue = currValue;
       }
    }

    if (!ros::ok()){
       break;
    }

   ROS_INFO_STREAM("Selected index " << maxIndex << " with value of " << maxValue);
   //Add support
   supports.push_back(maxIndex);

   //Created dictA with only the cols of the support
   Eigen::MatrixXf dictASupport(dictA.rows(), supports.size()); //Matrix of A ONLY containing the support
   int idx = 0;
   for (int& support : supports){
      dictASupport.col(idx) = dictA.col(support);
      idx++;
   }

   //Find least-squares
   Eigen::MatrixXf xTemp = dictASupport.bdcSvd(ComputeThinU | ComputeThinV).solve(targetY);
   // cout << "xTemp: \n" << xTemp << "\n";

   idx=0;
   // cout << "supports: \n";

   ofstream myfile;
   myfile.open("OMP_loop_update.txt");

   for (int& support : supports){
     // cout << support << "\n";
     myfile << "set xHat[" << support << "] to " << xTemp(idx) << "\n";
      xHat(support) = xTemp(idx);
      myfile << xHat(support) << "\n";
      idx++;
   }
   myfile << "updated xhat: \n" << xHat << "\n";
   myfile.close();
   error = targetY - (dictA * xHat);

   float oldErrorNorm = errorNorm;
   errorNorm = error.norm();
   ROS_DEBUG_STREAM("Error norm " << errorNorm << " for iteration " << iteration);
   float normDelta = oldErrorNorm - errorNorm;
   iteration++;
   // xHatOut = xHat;
   // errorOut = error;
   if (errorNorm < errorThresh){
      ROS_INFO_STREAM("Found solution with error norm of " << errorNorm << " which is below the threshold of " << errorThresh);
      xHatOut = xHat;
      errorOut = error;
      ofstream myfile;
      myfile.open("OMP_break_loop.txt");
      myfile << "break loop xHat: \n" << xHat << "\n";
      myfile << "break loop xHatOut: \n" << xHatOut << "\n";
      myfile.close();

      break;
   }
   else if (iteration > maxIterations){
      ROS_WARN_STREAM("Maximium iterations of " << maxIterations << " exceeded. Breaking");
      ofstream myfile;
      myfile.open("OMP_break_loop.txt");
      myfile << "final xHat: \n" << xHat << "\n";
      myfile.close();
      xHatOut = xHat;
      errorOut = error;
      break;
   }
  }

  // cout << "final xhat: \n" << xHat << "\n";
  // cout << "result is " << result.cols() << " columns by " << result.rows() << "rows \n";

  // myfile.open("debug4.txt");
  // myfile << "final xHat before saving into result: \n" << xHat << "\n";
  // myfile.close();
  // result.col(0) = xHat;
  // result.col(1).head(dictA.rows()) = error;

  ofstream myfile;
  myfile.open("OMP_final_return.txt", std::ios::app);


  myfile << "xHat: \n" << xHat << "\n error: \n" << error << "\n";
  myfile << "xHat OUT: \n" << xHatOut << "\n error OUT: \n" << errorOut << "\n";
  myfile.close();
  return xHat.maxCoeff();
}

std::vector<std::vector<int>> CoarseDM::getSamplePoints(const cv::Mat &img) {
  std::vector<std::vector<int>> result;

    // basic histogram code from:
    // https://docs.opencv.org/3.4/d8/dbc/tutorial_histogram_calculation.html
    // int histSize = 256;
    // float range[] = {0,1};
    // const float* histRange = { range };
    // bool uniform = true, accumulate = false;
    // cv::Mat hist;
    // calcHist(&img, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
    // int hist_w = 512, hist_h = 400;
    // int bin_w = cv::cvRound( (double) hist_w/histSize);
    // cv::Mat histImg(hist_h, hist_w, CV_8UC3, Scalar(0,0,0));
    // cv::normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, cv::Mat() );
    // cv::imshow("histogram?", histImg);
    // cv::waitKey(1);result(i) = img.at<float>(curY, curX);

    // PDF based on image intensity (?)
    float intensity_thresh = 0.6;   // arbitrary; if too high, will not reach cap!
    int sample_cap = 4000;          // arbitrary; paper says this should be 1/17th of
                                    //            total pixels in image? unsure
    int num_samples = 0;
    while (num_samples < sample_cap) {
      // TODO: determine whether to generate x and y OR use hilbert space filling curve
      int curX = rand() % img.cols;
      int curY = rand() % img.rows;
      float curVal = img.at<float>(curY, curX);
      if (curVal > intensity_thresh) {
        std::vector<int> sample_point = {curX, curY};
        result.push_back(sample_point);
        num_samples++;
      }
    }
  return result;
}

// given reference image, returns sample points based on
// probability density function using intensity as threshold
// std::vector<std::vector<int>> getSamplePoints(cv::Mat img) {
//   std::vector<std::vector<int>> result;
//
//   // basic histogram code from:
//   // https://docs.opencv.org/3.4/d8/dbc/tutorial_histogram_calculation.html
//   // int histSize = 256;
//   // float range[] = {0,1};
//   // const float* histRange = { range };
//   // bool uniform = true, accumulate = false;
//   // cv::Mat hist;
//   // calcHist(&img, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
//   // int hist_w = 512, hist_h = 400;
//   // int bin_w = cv::cvRound( (double) hist_w/histSize);
//   // cv::Mat histImg(hist_h, hist_w, CV_8UC3, Scalar(0,0,0));
//   // cv::normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, cv::Mat() );
//   // cv::imshow("histogram?", histImg);
//   // cv::waitKey(1);result(i) = img.at<float>(curY, curX);
//
//   // PDF based on image intensity (?)
//   float intensity_thresh = 0.6;   // arbitrary; if too high, will not reach cap!
//   int sample_cap = 4000;          // arbitrary; paper says this should be 1/17th of
//                                   //            total pixels in image? unsure
//   int num_samples = 0;
//   while (num_samples < sample_cap) {
//     // TODO: determine whether to generate x and y OR use hilbert space filling curve
//     int curX = rand() % img.cols;
//     int curY = rand() % img.rows;
//     float curVal = img.at<float>(curY, curX);
//     if (curVal > intensity_thresh) {
//       result.push_back(curVal);
//       num_samples++;
//     }
//   }
//   return result;
// }
