#include "ros/ros.h"
// #include "std_msgs/String.h"
#include <imaging_sonar_msgs/SonarImage.h>
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
  // send imaging sonar message to hensonLib functions
  // rosbag uses imaging_sonar_msgs/SonarImage messages
  printf("got a message!\n");
  auto sonar_intensities = msg->intensities;
  auto sonar_ranges = msg->ranges;
  // int range_length = sizeof(sonar_ranges)/sizeof(sonar_ranges[0]);
  int range_length = sonar_ranges.size();
  auto sonar_bearings = msg->azimuth_angles;
  // int bearing_length = sizeof(sonar_bearings)/sizeof(sonar_bearings[0]);
  int bearing_length = sonar_bearings.size();

  // intensities[0] = (bearing[0], ranges[0])
  // len(intensities) = len(ranges) *
  //         min(1,len(elevation_angles)) *
  //         len(bearings) * data_size
  int sonar_data[bearing_length][range_length];
  // int sonar_data[range_length][bearing_length];
  int index;
  for (int i = 0; i < bearing_length; i++) {
    for (int j = 0; j < range_length; j++) {
      // sonar_data[i][j] = sonar_intensities[j * range_length + i * bearing_length];
      index = j + (i * range_length);
      sonar_data[i][j] = sonar_intensities[index];
      // cout << index << endl;
    }
  }
  Mat out(bearing_length, range_length, CV_8U, sonar_data);
  // cv::resize(out, out, cv::Size(), 10.75, 10.75);
  // cv::namedWindow("out", 0);
  // cv::resizeWindow("out", 800,800);
  cv::imshow("out", out);
  cv::waitKey(10);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "sonarlistener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/oculus/sonar_image", 1000, sonarCallback); // TODO change first arg to name of sonar topic

  ros::spin();

  return 0;
}
