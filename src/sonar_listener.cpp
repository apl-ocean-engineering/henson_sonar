#include "ros/ros.h"
// #include "std_msgs/String.h"
#include <imaging_sonar_msgs/SonarImage.h>
#include "image_proc/sonarImageProc.h"
// imaging_sonar_msgs::SonarImage msg
#include "henson_sonar/coarseDM.h"
// #include <Eigen3/Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <iostream>
// #include <valarray>
#include <thread>

using namespace cv;
using namespace std;

const int OMP_THREADS = 10;

bool first;
cv::Mat prevCartesian;
SonarImageProc image_proc;
CoarseDM coarse_dm;
int frame_num;

void threadedOMP(std::vector<std::vector<int>> points, const cv::Mat& curCartesian, const cv::Mat& prevCartesian, cv::Mat& omp_collage) {
  for (std::vector<int>& point : points) {
    int curX = point[0];
    int curY = point[1];
    // cout << "omp at point: " << curX << ", " << curY << "\n";
    // cout << "img width : " << float_cartesian.cols << endl;
    // cout << "img height: " << float_cartesian.rows << endl;
    // cout << "collage width : " << omp_collage.cols << endl;
    // cout << "collage height: " << omp_collage.rows << endl;
    Eigen::VectorXf target_gamma = coarse_dm.getGamma(curX, curY, curCartesian);
    Eigen::Matrix<float, Dynamic, Dynamic> dict_matrix = coarse_dm.dictionaryMatrix(curX, curY, curCartesian, prevCartesian);

    Eigen::VectorXf error;
    Eigen::VectorXf xHat;

    float omp_max = coarse_dm.getTargetErrorOMP(dict_matrix, target_gamma, xHat, error);

    omp_collage.at<float>(curY, curX) = omp_max;
  }
}


void sonarCallback(const imaging_sonar_msgs::SonarImage::ConstPtr& msg)
{
  // // send imaging sonar message to hensonLib functions
  // // rosbag uses imaging_sonar_msgs/SonarImage messages
  // // printf("got a message!\n");
  // global boolean first;

  // SonarImageProc image_proc;
  image_proc.parseImg(msg);
  // cv::Mat polar = image_proc.polarImage();
  cv::Mat cartesian = image_proc.cartesianImage();
  cv::Mat float_cartesian;
  cartesian.convertTo(float_cartesian, CV_32FC1);
  float_cartesian /= 255;
  cv::imshow("float cartesian", float_cartesian);
  cv::waitKey(1);
  cv::Mat curCartesian;
  cv::copyMakeBorder(float_cartesian, curCartesian, 41, 41, 41, 41, BORDER_CONSTANT, 0.0);

  // convert image to float values
  // cv::Mat floatImg;
  // curCartesian.convertTo(floatImg, CV_32FC1);
  // floatImg /= 255;
  cv::imshow("cartesian", curCartesian);
  cv::waitKey(1);

  if (first) {
    prevCartesian = curCartesian;
    first = false;
  } else {

    // int count0 = 0;
    // int count1 = 0;
    // int count2 = 0;
    // int count3 = 0;
    // for (int i = 0; i < floatImg.rows; i++) {
    //   for (int j = 0; j < floatImg.cols; j++) {
    //     int curX = j;
    //     int curY = i;
    //     float curVal = floatImg.at<float>(curY, curX);
    //     if (0 > curVal && curVal < 0.25) {
    //       count0++;
    //     } else if (0.25 >= curVal && curVal < 0.5) {
    //       count1++;
    //     } else if (0.5 >= curVal && curVal < 0.75) {
    //       count2++;
    //     } else {
    //       count3++;
    //     }
    //   }
    // }
    // cout << "basic distribution of points: \n";
    // cout << "between 0 and 0.25:   " << count0 << "\n";
    // cout << "between 0.25 and 0.5: " << count1 << "\n";
    // cout << "between 0.5 and 0.75: " << count2 << "\n";
    // cout << "between 0.75 and 1.0: " << count3 << "\n";

    cv::Mat omp_collage = Mat(curCartesian.rows, curCartesian.cols, CV_32FC1);

    std::vector<std::vector<int>> sample_points = coarse_dm.getSamplePoints(curCartesian);

    // divvy up points for each OMP thread to work on
    std::vector<std::vector<std::vector<int>>> thread_jobs;
    int thread_job_size = (int)(sample_points.size() / OMP_THREADS);
    int last_idx;
    for (int i = 0; i < OMP_THREADS; i++) {
      int begin_idx = i*thread_job_size;
      int end_idx = ((i+1)*thread_job_size)-1;
      std::vector<std::vector<int>> points;
      for (int j = begin_idx; j <= end_idx; j++) {
        points.push_back(sample_points[j]);
      }
      thread_jobs.push_back(points);
      last_idx = end_idx;
    }
    // in case OMP_THREADS doesn't divide evenly into the number of sample points,
    // just dump the rest into the last thread (potentially bad solution)
    for (int i = last_idx+1; i < sample_points.size(); i++) {
      thread_jobs[thread_jobs.size()-1].push_back(sample_points[i]);
    }

    // make a bunch of threads, each one working on it's own set of sample points
    // std::vector<std::thread> threads;
    std::thread threads[OMP_THREADS];
    for (int i = 0; i < OMP_THREADS; i++) {
      std::vector<std::vector<int>> thread_points = thread_jobs[i];
      threads[i] = std::thread(threadedOMP, thread_points, std::ref(curCartesian), std::ref(prevCartesian), std::ref(omp_collage));
    }
    // wait for all threads to finish
    for (int i = 0; i < OMP_THREADS; i++) {
      threads[i].join();
    }

    // interpolate
    cv::Mat interpolation_img = omp_collage.clone();
    for (std::vector<int>&point : sample_points) {
      int curX = point[0];
      int curY = point[1];
      coarse_dm.interpolateOMPimage(omp_collage, interpolation_img, curX, curY);
    }

    // FOR SAVING OUTPUT IMAGES
    std::string filename = "src/henson_sonar/output/frame" + std::to_string(frame_num) + ".png";
    cv::Mat save_omp_collage;
    omp_collage.convertTo(save_omp_collage, CV_8UC1);
    save_omp_collage *= 255;
    cv::imwrite(filename, save_omp_collage);

    filename = "src/henson_sonar/output/interp" + std::to_string(frame_num) + ".png";
    cv::Mat save_interpolation;
    interpolation_img.convertTo(save_interpolation, CV_8UC1);
    save_interpolation *= 255;
    cv::imwrite(filename, save_interpolation);
    // cv::imshow("OMP output", omp_collage);
    // cv::waitKey(1);

  }
  prevCartesian = curCartesian;
  frame_num++;
}
int main(int argc, char **argv)
{

  first = true;

  frame_num = 0;

  ros::init(argc, argv, "sonarlistener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/oculus/sonar_image", 1000, sonarCallback);

  ros::spin();

  return 0;
}
