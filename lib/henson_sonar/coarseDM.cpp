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
#include <stdint.h>
#include <unordered_set>

#include <ros/console.h>

using namespace Eigen;
using namespace cv;
using namespace std;

const float OMP_ERROR_THRESH = 0.10;
const int   OMP_ITERATION_MAX = 10;
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
Eigen::Matrix<float, Dynamic, Dynamic> CoarseDM::dictionaryMatrix(int pixelX, int pixelY, const cv::Mat& imgRef) {
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
int CoarseDM::getTargetErrorOMP(const Matrix<float, Dynamic, Dynamic>& dictA, const VectorXf& targetY, VectorXf& xHatOut, VectorXf& errorOut) {
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
  // myfile.open("OMP_function_return.txt", std::ios::app);
  //
  //
  // myfile << "xHat: \n" << xHat << "\n error: \n" << error << "\n";
  // myfile << "xHat OUT: \n" << xHatOut << "\n error OUT: \n" << errorOut << "\n";


  // float result = max(xHat.maxCoeff(), abs(xHat.minCoeff()));
  int max_idx = 0;
  float max_val = 0;
  for (int i = 0; i < xHat.size(); i++) {
    float curVal = xHat(i);
    // filter out bad results
    if (curVal > 100) xHat(i) = 0;
    if (curVal < -100) xHat(i) = 0;

    curVal = xHat(i);
    if (curVal > max_val) {
      max_idx = i;
      max_val = xHat(i);
    }
  }

  // myfile << "return value: " << max_idx << endl;
  // myfile.close();
  return max_idx;
}

// input:
//  img   OMP image (CV_32FC1)
// output:
//  array of sample points (each point: [x,y])
std::vector<std::vector<int>> CoarseDM::getSamplePoints(const cv::Mat &img) {
  std::vector<std::vector<int>> result;

    int num_samples = 0;
    // while (num_samples < sample_cap) {
    while (num_samples < OMP_NUM_SAMPLES) {
      // TODO: determine whether to generate x and y OR use hilbert space filling curve
      int border_size = 41;
      int curX = (rand() % (img.cols-(border_size*2)))+border_size;
      int curY = (rand() % (img.rows-(border_size*2)))+border_size;
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

      // map 0-1680 to 10-1
      // higher intensity -> closer to 1 -> more likely rand() % mod_val == 0
      // to make lower intensity points less likely to be saved, just increase the third number
      // in the call to rangeMap
      // int mod_val = rangeMap(curVal, 0, 1680, 1, 11);
      // mod_val = flipRange(mod_val, 1, 11);
      // cout << mod_val <<  endl;
      // int random = rand() % mod_val;
      // cout << "val: " << curVal;
      // cout << random << endl;
      // if (random == 0) save = true;

      int random = rand() % 100;
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
    // ofstream myfile;
    // myfile.open("sample_points.txt", std::ios::app);
    // myfile << "points" << endl;
    // for (int i = 0; i < result.size(); i++) {
    //   myfile << result[i][0] << ", " << result[i][1] << endl;
    // }
    // myfile.close();
  return result;
}

struct VectorHash {
    size_t operator()(const std::vector<int>& v) const {
        std::hash<int> hasher;
        size_t seed = 0;
        for (int i : v) {
            seed ^= hasher(i) + 0x9e3779b9 + (seed<<6) + (seed>>2);
        }
        return seed;
    }
};

// void CoarseDM::interpolateOMPimage(const cv::Mat& img, cv::Mat& out, int x, int y) {
// input:
//  img   reference OMP image (32SC1)
//  out   interpolated image  (32SC1)
//  x     x coordinate of point of interest
//  y     y coordinate of point of interest
int CoarseDM::interpolateOMPimage(const cv::Mat& img, cv::Mat& out, int x, int y, std::vector<std::vector<int>> sample_points) {
  // make histogram of sample points within 13 x 13 area around point
  std::unordered_set<std::vector<int>, VectorHash> sample_points_set;
  std::copy(sample_points.begin(), sample_points.end(), std::inserter(sample_points_set, sample_points_set.end()));
  // get list of all sample points in 13 x 13 area around point
  std::vector<std::vector<int>> window_points;
  for (int j = y-6; j <= y+6; j++) {
    for (int i = x-6; i <= x+6; i++) {
      std::vector<int> cur_point = {i, j};
      bool valid = false;
      if (sample_points_set.find(cur_point) != sample_points_set.end()) valid = true;
      if (valid) window_points.push_back(cur_point);
    }
  }

  // cout << "num points in window: " << window_points.size() << endl;

  int imgMax = 0;
  int imgMin = 1681;
  for (std::vector<int>&point : window_points) {
    int curVal = img.at<int>(point[1], point[0]);
    if (curVal > imgMax) imgMax = curVal;
    if (curVal < imgMin) imgMin = curVal;
  }


  int histSize = 10; // number of bins: not sure how to configure
  int binWidth = (int) (imgMax-imgMin) / histSize;
  if (binWidth == 0) binWidth = 1;

  // cout << "min: " << imgMin << "\n";
  // cout << "max: " << imgMax << "\n";
  // cout << "bin width: " << binWidth << "\n";

  std::vector<std::vector<int>> hist(histSize);

  // ofstream myfile;
  // myfile.open("OMP_interpolation_debug.txt", std::ios::app);
  // myfile << "RAW IMAGE DATA: \n";

  // int nonzero = 0;

  int val, hist_index;
  for (std::vector<int>&point : window_points) {
    val = img.at<int>(point[1], point[0]);
    hist_index = (int) ((val-imgMin) / binWidth);
    if (hist_index >= histSize) hist_index = histSize-1;
    if (hist_index < 0) hist_index = 0;
    hist[hist_index].push_back(val);
  }

  // find bin with most points, set all points in 13x13 window to avg value of bin
  int max_bin_index = 0;
  for (int i = 0; i < hist.size(); i++) {
    if (hist[i].size() >= hist[max_bin_index].size()) {
      max_bin_index = i;
    }
  }

  // if (hist[max_bin_index].size() == 0) max_bin_index = 0;

  int binSum = 0;
  for (int i = 0; i < hist[max_bin_index].size(); i++) {
    binSum += hist[max_bin_index][i];
  }
  // cout << "max bin sum = " << binSum << endl;
  int avg = (int)(binSum/hist[max_bin_index].size());

  for (int j = y-6; j <= y+6; j++) {
    for (int i = x-6; i <= x+6; i++) {
      // cout << "set point " << i << ", " << j << " to " << avg << endl;
      out.at<int>(j,i) = avg;
    }
  }

  int test_x = avg % 41;

  int test_y = (int) (avg / 41);
  // cout << "nonzero: " << nonzero << endl;
  // cout << "test x: " << test_x << endl;
  // cout << "test y: " << test_y << endl;
  //
  // cout << "avg of " << avg << endl;
  // myfile.close();

  return avg;
}

// map input value from one range to another
int CoarseDM::rangeMap(int in, int in_start, int in_end, int out_start, int out_end) {
  int in_range = in_end - in_start;
  int out_range = out_end - out_start;

  return (int)(in - in_start)*out_range / in_range + out_start;

}

// flip number in range
int CoarseDM::flipRange(int in, int min, int max) {
  return (max + min) - in;
}

// compares two coarse depth map images (post interpolation)
// input:
//  forwardDM   forward depth map image (32SC1)
//  backDM      backward depth map image (32SC1)
// output:
//  forwardDM   forward DM is corrected and modified
void CoarseDM::compareDM(cv::Mat& forwardDM, const cv::Mat& backwardDM) {
  // comparison for every pixel position ξ ∈ X (every pixel in image? or sample points?)
  // doing all pixels for now

  cv::Mat valid_estimates = Mat(forwardDM.rows, forwardDM.cols, CV_8UC1, Scalar(0));
  vector<Point> valid_points;
  vector<Point> invalid_points;

  for (int i = 0; i < forwardDM.cols; i++) {
    for (int j = 0; j < forwardDM.rows; j++) {

      int fwd_val = forwardDM.at<int>(j, i);
      int fwd_x_shift = (fwd_val % 41) - 20;
      int fwd_y_shift = 20 - ((int) fwd_val / 41);

      int bwd_val = backwardDM.at<int>(j-fwd_y_shift, i-fwd_x_shift);
      int bwd_x_shift = (bwd_val % 41) - 20;
      int bwd_y_shift = 20 - ((int) bwd_val / 41);

      bool valid = ((abs(bwd_x_shift + fwd_x_shift) + abs(bwd_y_shift + fwd_y_shift)) > 2);
      if (valid) valid_points.push_back({i, j});
      else invalid_points.push_back({i, j});
      valid_estimates.at<uchar>(j, i) = (int) valid;

      // if not valid, find nearest accepted measurement
      // idea:
      //  make binary image same size as depth map, init to all zeros
      //  fill in this image with 1's anywhere there is an accepted measurement
      //  use opencv's findNonZero function to get all accepted points
      //  for each point that isn't aceepted (= 0), find nearest accepted point
      //    (just use pythagorean theorem for this)

    }
  }

  for (Point& p : invalid_points) {
    // find nearest valid point
    // NOTE: extremely inefficient. need to implement spiral search or something better
    int dist, min_idx;
    dist = pow(valid_points[0].x - p.x, 2) + pow(valid_points[0].y - p.y, 2);
    min_idx = 0;
    for (int i = 1; i < valid_points.size(); i++) {
      int cur = pow(valid_points[i].x - p.x, 2) + pow(valid_points[i].y - p.y, 2);
      if (cur < dist) {
        cur = dist;
        min_idx = i;
      }
    }

    forwardDM.at<int>(p.y, p.x) = forwardDM.at<int>(valid_points[min_idx]);
  }
}

// (scale as in intensity, not size)
cv::Mat CoarseDM::scaleImg(cv::Mat& img) {
  // double min, max;
  // cv::minMaxLoc(img, &min, &max);
  // int imgMin = (int)min;
  // int imgMax = (int)max;
  // cv::Mat scale;
  // img.convertTo(scale, CV_32FC1);
  // scale /= imgMax;
  // cv::Mat ret;
  // scale.convertTo(ret, CV_8UC1);
  // ret *= 255;
  // cout << "max: " << imgMax << endl;
  // // cv::Mat ret;
  // // img.convertTo(ret, CV_8UC1);
  // // ret = (int)(((float)ret / imgMax) * 255);
  cv::Mat ret;
  cv::normalize(img, ret, 0, 255, NORM_MINMAX, CV_8UC1);
  return ret;
}

int CoarseDM::hsfc_last2bits(int x) { return x&3; }

Point CoarseDM::hsfc_indexToXY(int hindex, int N) {
  Point positions[4] = {{0,0}, {0,1}, {1,1}, {1,0}};
  Point tmp = positions[hsfc_last2bits(hindex)];
  hindex = (hindex >> 2);
  int x = tmp.x;
  int y = tmp.y;
  for (int n = 4; n <= N; n*= 2) {
    int n2 = n/2;
    switch (hsfc_last2bits(hindex)) {
      int swap;
      case 0:
        swap = x;
        x = y;
        y = swap;
        break;
      case 1:
        x = x;
        y = y + n2;
        break;
      case 2:
        x = x + n2;
        y = y + n2;
        break;
      case 3:
        swap = y;
        y = (n2-1) - x;
        x = (n2-1) - swap;
        x = x+n2;
        break;
    }
    hindex = (hindex >> 2);
  }
  return {x,y};
}
