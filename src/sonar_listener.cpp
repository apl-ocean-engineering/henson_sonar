#include "ros/ros.h"
// #include "std_msgs/String.h"
#include <imaging_sonar_msgs/SonarImage.h>
#include "image_proc/sonarImageProc.h"
// imaging_sonar_msgs::SonarImage msg
#include <henson_sonar/coarseDM.h>
// #include <Eigen3/Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

void sonarCallback(const imaging_sonar_msgs::SonarImage::ConstPtr& msg)
{
  // // send imaging sonar message to hensonLib functions
  // // rosbag uses imaging_sonar_msgs/SonarImage messages
  // // printf("got a message!\n");
  SonarImageProc image_proc;
  image_proc.parseImg(msg);
  cv::Mat polar = image_proc.polarImage();
  cv::Mat cartesian = image_proc.cartesianImage();
  cv::imshow("polar", polar);
  cv::imshow("cartesian", cartesian);
  cv::waitKey(1);
}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "sonarlistener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/oculus/sonar_image", 1000, sonarCallback); // TODO change first arg to name of sonar topic

  ros::spin();

  return 0;
}
