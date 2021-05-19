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
std::vector<cv::Mat> img_queue;

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

    omp_collage.at<int>(curY, curX) = omp_max;
  }
}


void sonarCallback(const imaging_sonar_msgs::SonarImage::ConstPtr& msg)
{

  image_proc.parseImg(msg);
  // cv::Mat polar = image_proc.polarImage();
  cv::Mat cartesian = image_proc.cartesianImage(); // unsigned char
  cv::Mat float_cartesian;
  cartesian.convertTo(float_cartesian, CV_32FC1);
  float_cartesian /= 255;
  // cv::imshow("float cartesian", float_cartesian);
  // cv::waitKey(1);
  cv::Mat curCartesian;
  cv::copyMakeBorder(float_cartesian, curCartesian, 41, 41, 41, 41, BORDER_CONSTANT, 0.0);
  int img_q_size = 24;

  if (frame_num > 102) {
    img_queue.push_back(curCartesian);
    if (img_queue.size() >= img_q_size) {
      cout << "image queue filled, processing " << img_queue.size() << " images\n";
      while (img_queue.size() != 0) {
        curCartesian = img_queue.back();
        img_queue.pop_back();

        // save original image
        std::string filename = "src/henson_sonar/output/frame" + std::to_string(save_frame_num) + ".png";
        cv::Mat save_cur_cartesian;
        curCartesian.convertTo(save_cur_cartesian, CV_8UC1);
        save_cur_cartesian *= 255;
        cv::imwrite(filename, save_cur_cartesian);

        if (first) {
          prevCartesian = curCartesian;
          first = false;
        } else {

          std::chrono::time_point<std::chrono::system_clock> start, end;
          std::chrono::duration<double> elapsed_time;

          cv::Mat omp_collage = Mat(curCartesian.rows, curCartesian.cols, CV_32SC1);

          start = std::chrono::system_clock::now();
          std::vector<std::vector<int>> sample_points = coarse_dm.getSamplePoints(curCartesian);
          end = std::chrono::system_clock::now();
          elapsed_time = end - start;
          cout << "sample points elapsed time: " << elapsed_time.count() << "s\n";

          // divvy up points for each OMP thread to work on
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

          // make a bunch of threads, each one working on it's own set of sample points
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

          // BAD FIX: eliminate outlier values in omp collage
          // for (int i = 0; i < omp_collage.cols; i++) {
          //   for (int j = 0; j < omp_collage.rows; j++) {
          //     float val = omp_collage.at<float>(j, i);
          //     if (val > 100) omp_collage.at<float>(j,i) = 0;
          //     if (val < -100) omp_collage.at<float>(j,i) = 0;
          //   }
          // }

          end = std::chrono::system_clock::now();
          elapsed_time = end - start;
          cout << "OMP finished in: " << elapsed_time.count() << "s\n";

          // save OMP collage image
          filename = "src/henson_sonar/output/omp" + std::to_string(save_frame_num) + ".png";
          // cv::Mat save_omp_collage;
          // omp_collage.convertTo(save_omp_collage, CV_8UC1);
          // save_omp_collage *= 255;
          // cv::imwrite(filename, save_omp_collage);
          cv::imwrite(filename, omp_collage);

          // interpolate
          start = std::chrono::system_clock::now();
          cv::Mat interpolation_img = Mat(curCartesian.rows, curCartesian.cols, CV_32SC1);
          for (std::vector<int>&point : sample_points) {
            int x = point[0];
            int y = point[1];
            // cout << "current point: " << x << ", " << y << endl;
            int val = coarse_dm.interpolateOMPimage(omp_collage, interpolation_img, x, y);
            // cout << "point interpolation finished" << endl;
          }
          end = std::chrono::system_clock::now();
          elapsed_time = end - start;
          cout << "interpolation finished in: " << elapsed_time.count() << "s\n";

          // save interpolation image
          filename = "src/henson_sonar/output/interp" + std::to_string(save_frame_num) + ".png";
          cv::Mat save_interpolation;
          interpolation_img.convertTo(save_interpolation, CV_8UC1);
          // save_interpolation *= 255;
          cv::imwrite(filename, save_interpolation);
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

  first = true;

  frame_num = 0;
  save_frame_num = 0;

  ros::init(argc, argv, "sonarlistener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/oculus/sonar_image", 1000, sonarCallback);
  // ros::Subscriber sub = n.subscribe("/oculus/sonar_image", 1000, photoCallback);

  ros::spin();

  return 0;
}
