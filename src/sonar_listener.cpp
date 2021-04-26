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

using namespace cv;
using namespace std;

bool first;
cv::Mat prevCartesian;
SonarImageProc image_proc;
CoarseDM coarse_dm;
int frame_num;

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
  // cv::imshow("polar", polar);
  cv::Mat curCartesian;
  cv::copyMakeBorder(cartesian, curCartesian, 41, 41, 41, 41, BORDER_CONSTANT, 0);

  // convert image to float values
  cv::Mat floatImg;
  curCartesian.convertTo(floatImg, CV_32FC1);
  floatImg /= 255;
  cv::imshow("cartesian", floatImg);
  cv::waitKey(1);

  if (first) {
    prevCartesian = floatImg;
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


    // get OMP output of points in the image
    cv::Mat omp_collage = Mat(curCartesian.rows, curCartesian.cols, CV_32FC1);
    // TODO: change nested for-loops to function call:
    std::vector<std::vector<int>> sample_points = coarse_dm.getSamplePoints(curCartesian);
    for (int k = 1; k < 7; k++) {
      for (int l = 1; l < 16; l++) {
        int pixelX = 41 * l;
        int pixelY = 41 * k;

        // coarse depth map
        Eigen::VectorXf target_gamma = coarse_dm.getGamma(pixelX, pixelY, floatImg);
        // cout << "target: \n" << target_gamma << "\n";
        Eigen::Matrix<float, Dynamic, Dynamic> dict_matrix = coarse_dm.dictionaryMatrix(pixelX, pixelY, floatImg, prevCartesian);
        // Eigen::VectorXf error(target_gamma.size());
        // Eigen::VectorXf xHat(dict_matrix.cols());
        // xHat.fill(0);
        // error.fill(0);

        // Eigen::VectorXf error;
        // Eigen::VectorXf xHat;

        // CURRENTLY NOT WORKING, FIX LATER
        // coarse_dm.getTargetErrorOMP(dict_matrix, target_gamma, xHat, error);
        float omp_max = coarse_dm.getTargetErrorOMP(dict_matrix, target_gamma);

        // cout << "omp output: \n" << omp_output << "\n";
        // cout << "omp error: \n" << omp_output.col(1) << "\n";
        // int idx = 0;
        // float omp_image_data[41][41];
        // for (int i = 0; i < 41; i++) {
        //   for (int j = 0; j < 41; j++) {
        //     omp_image_data[i][j] = omp_result(idx);
        //     idx++;
        //   }
        // }

        // UNCOMMENT LATER
        // ofstream myfile;
        // myfile.open("debug5.txt");
        // myfile << "xhat from omp: \n" << xHat << "\n";
        // myfile.close();
        // int idx = 0;
        // for (int i = 0; i < 41; i++) {
        //   for (int j = 0; j < 41; j++) {
        //     int curX = pixelX - 20 + j;
        //     int curY = pixelY - 20 + i;
        //     // cout << curX << "\n";
        //     // cout << curY << "\n";
        //     // cout << "omp: " << omp_xhat(idy) << "\n";
        //     omp_collage.at<float>(curY, curX) = xHat(idx);
        //
        //     idx++;
        //   }
        // }

      }
    }
    // cv::Mat omp_image = Mat(41, 41, CV_32FC1, &omp_image_data);
    // cv::Mat dst;
    // cv::resize(omp_image, dst, cv::Size(omp_image.cols * 10, omp_image.rows * 10));

    // FOR SAVING OUTPUT IMAGES
    // std::string filename = "src/henson_sonar/output/frame" + std::to_string(frame_num) + ".png";
    // cv::Mat save_omp_collage;
    // omp_collage.convertTo(save_omp_collage, CV_8UC1);
    // save_omp_collage *= 255;
    // cv::imwrite(filename, save_omp_collage);
    // cv::imshow("OMP output", omp_collage);
    // cv::waitKey(1);

  }
  prevCartesian = floatImg;
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
