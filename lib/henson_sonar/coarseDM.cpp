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
#include <algorithm>

#include <ros/console.h>

using namespace Eigen;
using namespace cv;
using namespace std;

const float OMP_ERROR_THRESH = 0.35;
const int   OMP_ITERATION_MAX = 6;
const int   OMP_NUM_SAMPLES = 15000;
const float OMP_SAMPLE_THRESH = 0.50;

CoarseDM::CoarseDM(){

}

// Given image, coordinates
// Outputs gamma vector centered at given coordinates of given image
// Assumes 13 x 13 area, single channel grayscale image (type 8UC1)
Eigen::VectorXf CoarseDM::getGamma(int x, int y, const cv::Mat& img) {
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
Eigen::Matrix<float, Dynamic, Dynamic> CoarseDM::dictionaryMatrix(int pixelX, int pixelY, const cv::Mat& imgTarget, const cv::Mat& imgRef) {
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

  // ROS_INFO("PROCESSING NEW IMAGE");

  Eigen::VectorXf xHat(dictA.cols());
  xHat.fill(0);

  Eigen::VectorXf error(dictA.rows()); // make error vector, duplicate values in targetY
  error = targetY/targetY.norm();

  // float errorThresh = 0.13; // placeholder
  // int maxIterations = 100;  // placeholder

  int iteration = 0;

   std::vector<int> supports;
  float errorNorm = error.norm();
   while (true){
   float maxValue = 0;
   int maxIndex = 0;
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

   // ROS_INFO_STREAM("Selected index " << maxIndex << " with value of " << maxValue);

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

   // ofstream myfile;
   // myfile.open("OMP_loop_update.txt");

   for (int& support : supports) {
     // cout << support << "\n";
     // myfile << "set xHat[" << support << "] to " << xTemp(idx) << "\n";
      xHat(support) = xTemp(idx);
      // myfile << xHat(support) << "\n";
      idx++;
   }
   // myfile << "updated xhat: \n" << xHat << "\n";
   // myfile.close();
   error = targetY - (dictA * xHat);

   float oldErrorNorm = errorNorm;
   errorNorm = error.norm();
   ROS_DEBUG_STREAM("Error norm " << errorNorm << " for iteration " << iteration);
   float normDelta = oldErrorNorm - errorNorm;
   iteration++;
   // xHatOut = xHat;
   // errorOut = error;
   if (errorNorm < OMP_ERROR_THRESH){
      // ROS_INFO_STREAM("Found solution with error norm of " << errorNorm << " which is below the threshold of " << OMP_ERROR_THRESH);


      xHatOut = xHat;
      errorOut = error;
      // ofstream myfile;
      // myfile.open("OMP_break_loop.txt");
      // myfile << "break loop xHat: \n" << xHat << "\n";
      // myfile << "break loop xHatOut: \n" << xHatOut << "\n";
      // myfile.close();

      break;
   }
   else if (iteration > OMP_ITERATION_MAX){
      // ROS_WARN_STREAM("Maximium iterations of " << OMP_ITERATION_MAX << " exceeded. Breaking");


      // ofstream myfile;
      // myfile.open("OMP_break_loop.txt");
      // myfile.close();
      // myfile << "final xHat: \n" << xHat << "\n";
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

  // ofstream myfile;
  // myfile.open("OMP_final_return.txt", std::ios::app);
  //
  //
  // myfile << "xHat: \n" << xHat << "\n error: \n" << error << "\n";
  // myfile << "xHat OUT: \n" << xHatOut << "\n error OUT: \n" << errorOut << "\n";
  // myfile.close();

  float result = max(xHat.maxCoeff(), abs(xHat.minCoeff()));
  if (result > 100) result = 0;
  if (result < -100) result = 0;

  // return max(xHat.maxCoeff(), abs(xHat.minCoeff()));
  return result;
}

std::vector<std::vector<int>> CoarseDM::getSamplePoints(const cv::Mat &img) {
  std::vector<std::vector<int>> result;

    // put dummy point into result
    // std::vector<int> dummy = {0, 0};
    // result.push_back(dummy);

    int num_samples = 0;
    // while (num_samples < sample_cap) {
    while (num_samples < OMP_NUM_SAMPLES) {
      // TODO: determine whether to generate x and y OR use hilbert space filling curve
      int curX = (rand() % (img.cols-82))+41;
      int curY = (rand() % (img.rows-82))+41;
      float curVal = img.at<float>(curY, curX);
      // if (curVal > OMP_SAMPLE_THRESH) {
      //   std::vector<int> sample_point = {curX, curY};
      //   result.push_back(sample_point);
      //   num_samples++;
      // }

      // the jankiest pdf ever
      bool save = false;
      // float mod_div = (int) curVal * 4;
      // if (mod_div == 0) mod_div = mod_div + 1;
      // int mod = (int) 12 / mod_div;
      // int random = rand() % mod;
      // if (random < 3) {
      //   save = true;
      // }
      int random = rand() % 100;
      // if (random < (int)((curVal * 100)-1)) save = true;
      if (random < (int)(pow(100,curVal)-1)) save = true;

      if (save) {
        std::vector<int> sample_point = {curX, curY};
        bool found = false;
        int idx = 0;
        while (!found && idx < result.size()) {
          if (result[idx][0] == curX && result[idx][1] == curY) found = true;
          idx++;
        }
        if (!found) {
          result.push_back(sample_point);
          num_samples++;
        }
      }
      // std::vector<int> sample_point = {curX, curY};
      // if (std::find(result.begin(), result.end(), sample_point) != result.end()) {
      //   result.push_back(sample_point);
      //   num_samples++;
      // }



    }
  return result;
}

// void CoarseDM::interpolateOMPimage(const cv::Mat& img, cv::Mat& out, int x, int y) {
float CoarseDM::interpolateOMPimage(const cv::Mat& img, cv::Mat& out, int x, int y) {
  // make histogram of 13 x 13 area around point
  // basic histogram code from:
  // https://docs.opencv.org/3.4/d8/dbc/tutorial_histogram_calculation.html
  // get 13x13 window centered at x,y
  // cv::Range cols(x-6, x+6);
  // cv::Range rows(y-6, y+6);
  // cv::Mat window = img(rows, cols);
  // cout << "window width : " << window.cols << endl;
  // cout << "window height: " << window.rows << endl;
  float imgMin = 0.0;
  float imgMax = 0.0;
  // float imgMax = float.MAX
  // cv::minMaxLoc(window, &imgMin, &imgMax);
  // openCV's minMaxLoc doesn't work with negative numbers -> look manually
  for (int i = x-6; i <= x+6; i++) {
    for (int j = y-6; j <= y+6; j++) {
      float val = img.at<float>(j,i);
      if (val < imgMin) imgMin = val;
      if (val > imgMax) imgMax = val;
    }
  }
  // confine imgMin and imgMax within semi-acceptable range
  // issue due to OMP collage having extremely large numbers
  // (1.5 and -1.5 are completely arbitrary!)
  // if (imgMax > 100) imgMax = 1.5;
  // if (imgMin < -100) imgMin = -1.5;
  //

  int histSize = 20; // number of bins: not sure how to configure
  // float imageMax = (float)imgMax;
  // float imageMin = (float)imgMin;
  float binWidth = (imgMax+abs(imgMin)) / histSize; // range of each bin
  int bins_below_zero = (int) (abs(imgMin) / binWidth);

  cout << "min: " << imgMin << "\n";
  cout << "max: " << imgMax << "\n";
  cout << "bin width: " << binWidth << "\n";
  // float range[] = {0,1};
  // const float* histRange = { range };
  // bool uniform = true, accumulate = false;

  // trying out my own method of making a histogram
  std::vector<std::vector<float>> hist(histSize);

  // ofstream myfile;
  // myfile.open("OMP_interpolation_debug.txt", std::ios::app);
  // myfile << "RAW IMAGE DATA: \n";
  for (int i = x-6; i <= x+6; i++) {
    for (int j = y-6; j <= y+6; j++) {

      float val = img.at<float>(j,i);

      int hist_index = (int) (val / binWidth);
      // correct for bins below zero
      if (val >= 0) hist_index += bins_below_zero;
      if (val < 0)  hist_index += (bins_below_zero - 1);

      // catch min, max, and NaN values
      if (hist_index >= histSize) hist_index = histSize-1;
      if (hist_index <= 0) hist_index = 0;

      // cout << "at: " << i << ", " << j << "\n";
      // cout << "val: " << val << "\n";
      // cout << "go into hist bin " << hist_index << "\n";

      // myfile << "go into hist bin " << hist_index << "\n";
      // myfile << "at pixel (" << i << "," << j << ")\n";
      // myfile << "val: " << val << "\n";

      hist[hist_index].push_back(val);

    }
  }

  // find bin with most points, set all points in 13x13 window to avg value of bin
  int max_bin_index = 0;
  for (int i = 0; i < hist.size(); i++) {
    if (hist[i].size() >= hist[max_bin_index].size()) {
      max_bin_index = i;
    }
  }
  // test

  float binSum = 0;
  for (int i = 0; i < hist[max_bin_index].size(); i++) {
    binSum += hist[max_bin_index][i];
  }
  // TODO: check proper value in binSum
  cout << "max bin sum = " << binSum << endl;
  float avg = binSum/hist[max_bin_index].size();

  float binMin = max_bin_index * binWidth;
  float binMax = (max_bin_index+1) * binWidth;

  // myfile.open("OMP_interpolation_debug.txt");

  // myfile << "HISTOGRAM SETUP:\n";
  // myfile << "number of bins: " << histSize << "\n";
  // myfile << "bin width: " << binWidth << "\n";
  // myfile << "HISTOGRAM DATA:\n";
  // myfile << "min bin index: " << max_bin_index << "\n";
  // myfile << "max bin index: " << max_bin_index << "\n";
  // myfile << "min value: " << imgMin << "\n";
  // myfile << "max value: " << imgMax << "\n";
  // // myfile << "RAW DATA: \n";
  // // for (int i = 0; i < hist.size(); i++) {
  // //   for (int j = 0; j < hist[i].size(); j++) {
  // //     myfile << hist[i][j] << "\n";
  // //   }
  // // }
  //
  // // // myfile << "13x13 window histogram: \n" << hist << "\n";
  // myfile << "max bin: intensity range of " << binMin << " to " << binMax << "\n";
  // myfile << "with " << hist[max_bin_index].size() << " pixels\n";
  // myfile << "and average value of " << avg << "\n";
  // myfile.close();

  for (int i = x-6; i <= x+6; i++) {
    for (int j = y-6; j <= y+6; j++) {
      // cout << "set point " << i << ", " << j << " to " << avg << endl;
      out.at<float>(j,i) = avg;
    }
  }

  // mutiply avg by 169 to make it work for x and y?

  int test_x = (int) (avg * 169);
  test_x = test_x % 13;
  int test_y = (int) (avg * 169);
  test_y = (int) (test_y / 13);
  cout << "test x: " << test_x << endl;
  cout << "test y: " << test_y << endl;

  cout << "avg of " << avg << endl;

  return avg;
  // int hist_w = 512, hist_h = 400;
  // int bin_w = cv::cvRound( (double) hist_w/histSize);
  // cv::Mat histImg(hist_h, hist_w, CV_8UC3, Scalar(0,0,0));
  // cv::normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, cv::Mat() );
  // cv::imshow("histogram?", histImg);
  // cv::waitKey(1);result(i) = img.at<float>(curY, curX);
}

// void compareDM(const cv::Mat& forwardDM, const cv::Mat& backwardDM, cv::Mat& out)
