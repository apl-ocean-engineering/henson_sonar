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

#include <chrono>
#include <ctime>

using namespace cv;
using namespace std;

const int OMP_THREADS = 10;

bool first;
cv::Mat prevCartesian;
SonarImageProc image_proc;
CoarseDM coarse_dm;
int frame_num;
int save_frame_num;
int wait;
std::vector<cv::Mat> img_queue;
const int BORDER_WIDTH = 41;

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
    Eigen::Matrix<float, Dynamic, Dynamic> dict_matrix = coarse_dm.dictionaryMatrix(curX, curY, prevCartesian);

    Eigen::VectorXf error;
    Eigen::VectorXf xHat;

    int omp_max = coarse_dm.getTargetErrorOMP(dict_matrix, target_gamma, xHat, error);

    // ofstream myfile;
    // myfile.open("OMP_return_(in_thread).txt", std::ios::app);
    // myfile << omp_max << endl;
    // myfile.close();

    omp_collage.at<int>(curY, curX) = omp_max;

    // myfile.open("OMP_image_value_after_write_(in_thread).txt", std::ios::app);
    // myfile << "image " << std::to_string(save_frame_num) << " " << curX << ", "<< curY << " = " << omp_collage.at<int>(curY, curX) << endl;
    // myfile.close();

    // myfile.open("OMP_image_outliers_(in_thread).txt", std::ios::app);
    // for (int i = 0; i < omp_collage.cols; i++) {
    //   for (int j = 0; j < omp_collage.rows; j++) {
    //     if (omp_collage.at<int>(j,i) > 2000) {
    //       myfile << "image " << std::to_string(save_frame_num) << " " << i << ", "<< j << " = " << omp_collage.at<int>(j,i) << endl;
    //     }
    //   }
    // }
    // myfile.close();

  }
}

cv::Mat getCoarseDM(const cv::Mat& curCartesian, const cv::Mat& prevCartesian) {
  // save original image
  std::string filename = "/home/tanner/catkin_ws/src/henson_sonar/output/frame" + std::to_string(save_frame_num) + ".png";
  cv::Mat save_cur_cartesian;
  curCartesian.convertTo(save_cur_cartesian, CV_8UC1);
  save_cur_cartesian *= 255;
  cv::imwrite(filename, save_cur_cartesian);

  // get sample points
  std::chrono::time_point<std::chrono::system_clock> start, end;
  std::chrono::duration<double> elapsed_time;

  start = std::chrono::system_clock::now();
  std::vector<std::vector<int>> sample_points = coarse_dm.getSamplePoints(curCartesian);
  end = std::chrono::system_clock::now();
  elapsed_time = end - start;
  cout << "sample points elapsed time: " << elapsed_time.count() << "s\n";

  // divvy up points for threads to work on during OMP image generation
  start = std::chrono::system_clock::now();
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
  end = std::chrono::system_clock::now();
  elapsed_time = end - start;
  cout << "OMP thread jobs made in: " << elapsed_time.count() << "s\n";

  // make OMP image
  cv::Mat omp_collage = Mat(curCartesian.rows, curCartesian.cols, CV_32SC1, Scalar(0));
  start = std::chrono::system_clock::now();
  std::thread threads[OMP_THREADS];
  for (int i = 0; i < OMP_THREADS; i++) {
    std::vector<std::vector<int>> thread_points = thread_jobs[i];
    threads[i] = std::thread(threadedOMP, thread_points, std::ref(curCartesian), std::ref(prevCartesian), std::ref(omp_collage));
  }
  // wait for all threads to finish
  for (int i = 0; i < OMP_THREADS; i++) {
    threads[i].join();
  }
  end = std::chrono::system_clock::now();
  elapsed_time = end - start;
  cout << "OMP finished in: " << elapsed_time.count() << "s\n";

  // save OMP collage image
  filename = "/home/tanner/catkin_ws/src/henson_sonar/output/omp" + std::to_string(save_frame_num) + ".png";
  cv::Mat save = coarse_dm.scaleImg(omp_collage);
  cv::imwrite(filename, save);
  // cv::imwrite(filename, omp_collage);

  // interpolate OMP image
  start = std::chrono::system_clock::now();
  cv::Mat interpolation_img = Mat(curCartesian.rows, curCartesian.cols, CV_32SC1, Scalar(0));
  for (std::vector<int>&point : sample_points) {
    int x = point[0];
    int y = point[1];
    // cout << "current point: " << x << ", " << y << endl;
    int val = coarse_dm.interpolateOMPimage(omp_collage, interpolation_img, x, y, sample_points);
    // cout << "point interpolation finished" << endl;
  }
  end = std::chrono::system_clock::now();
  elapsed_time = end - start;
  cout << "interpolation finished in: " << elapsed_time.count() << "s\n";

  return interpolation_img;
}

bool once;

void sonarCallback(const imaging_sonar_msgs::SonarImage::ConstPtr& msg)
{

  if (once) {
    once = false;
    int n = 8;
    // Point[] square1_hfsc
    for (int i = 0; i < n*n; i++) {
      Point cur = coarse_dm.hsfc_indexToXY(i, n);
      cout << "x: " << cur.x << " y: " << cur.y << endl;
    }
  }


  image_proc.parseImg(msg);
  // cv::Mat polar = image_proc.polarImage();
  cv::Mat cartesian = image_proc.cartesianImage(); // unsigned char
  cv::Mat float_cartesian;
  cartesian.convertTo(float_cartesian, CV_32FC1);
  float_cartesian /= 255;
  // cv::imshow("float cartesian", float_cartesian);
  // cv::waitKey(1);
  cv::Mat curCartesian;
  cv::copyMakeBorder(float_cartesian, curCartesian, BORDER_WIDTH, BORDER_WIDTH, BORDER_WIDTH, BORDER_WIDTH, BORDER_CONSTANT, 0.0);
  int img_q_size = 5;

  if (frame_num > 80) {
    wait++;
    if (wait % 5 == 0) img_queue.push_back(curCartesian);
    // img_queue.push_back(curCartesian);
    if (img_queue.size() >= img_q_size) {
      cout << "image queue filled, processing " << img_queue.size() << " images\n";
      while (img_queue.size() != 0) {
        curCartesian = img_queue.back();
        img_queue.pop_back();

        if (first) {
          prevCartesian = curCartesian;
          first = false;
        } else {
          // get forward and backward DM
          cv::Mat fwdDM = getCoarseDM(curCartesian, prevCartesian);

          cv::Mat bwdDM = getCoarseDM(prevCartesian, curCartesian);

          // save initial forward coarse DM
          cv::Mat save;
          std::string filename = "/home/tanner/catkin_ws/src/henson_sonar/output/fwd_interp" + std::to_string(save_frame_num) + ".png";
          save = coarse_dm.scaleImg(fwdDM);
          cv::imwrite(filename, save);

          // save initial backward coarse DM
          filename = "/home/tanner/catkin_ws/src/henson_sonar/output/bwd_interp" + std::to_string(save_frame_num) + ".png";
          save = coarse_dm.scaleImg(bwdDM);
          cv::imwrite(filename, save);

          std::chrono::time_point<std::chrono::system_clock> start, end;
          std::chrono::duration<double> elapsed_time;
          start = std::chrono::system_clock::now();
          // compare to eliminate bad estimaes
          coarse_dm.compareDM(fwdDM, bwdDM);
          end = std::chrono::system_clock::now();
          elapsed_time = end - start;
          cout << "coarse DM comparison complete in: " << elapsed_time.count() << "s\n";

          // save final forward coarse DM
          filename = "/home/tanner/catkin_ws/src/henson_sonar/output/final_coarseDM" + std::to_string(save_frame_num) + ".png";
          save = coarse_dm.scaleImg(fwdDM);
          cv::imwrite(filename, save);

        }
        prevCartesian = curCartesian;
        save_frame_num++;
      }
    }


  }

  frame_num++;
}
int main(int argc, char **argv)
{

  once = true;
  first = true;

  frame_num = 0;
  save_frame_num = 0;
  wait = 0;

  ros::init(argc, argv, "sonarlistener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/oculus/sonar_image", 1000, sonarCallback);
  // ros::Subscriber sub = n.subscribe("/oculus/sonar_image", 1000, photoCallback);

  ros::spin();

  return 0;
}
