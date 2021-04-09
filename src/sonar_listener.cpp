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

using namespace cv;
using namespace std;

bool first;
cv::Mat prevCartesian;
SonarImageProc image_proc;
CoarseDM coarse_dm;

void sonarCallback(const imaging_sonar_msgs::SonarImage::ConstPtr& msg)
{
  // // send imaging sonar message to hensonLib functions
  // // rosbag uses imaging_sonar_msgs/SonarImage messages
  // // printf("got a message!\n");
  // global boolean first;

  // SonarImageProc image_proc;
  image_proc.parseImg(msg);
  // cv::Mat polar = image_proc.polarImage();
  cv::Mat curCartesian = image_proc.cartesianImage();
  // cv::imshow("polar", polar);
  cv::imshow("cartesian", curCartesian);
  cv::waitKey(1);

  // convert image to float values
  cv::Mat floatImg;
  curCartesian.convertTo(floatImg, CV_32FC1);

  int pixelX = 100;
  int pixelY = 100;

  if (first) {
    prevCartesian = floatImg;
    first = false;
  } else {
    // coarse depth map
    Eigen::VectorXf target_gamma = coarse_dm.getGamma(pixelX, pixelY, floatImg);
    Eigen::Matrix<float, Dynamic, Dynamic> dict_matrix = coarse_dm.dictionaryMatrix(pixelX, pixelY, floatImg, prevCartesian);
    // Eigen::Matrix<int, 169, 2> result = coarse_dm.getTargetErrorOMP(dict_matrix, target_gamma);
    Eigen::Matrix<float, Dynamic, Dynamic> result = coarse_dm.getTargetErrorOMP(dict_matrix, target_gamma);
  }
  prevCartesian = floatImg;
}
int main(int argc, char **argv)
{

  first = true;

  ros::init(argc, argv, "sonarlistener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/oculus/sonar_image", 1000, sonarCallback);

  ros::spin();

  return 0;
}
