#include "ros/ros.h"
// #include "std_msgs/String.h"
#include <imaging_sonar_msgs/SonarImage.h>
// imaging_sonar_msgs::SonarImage msg
#include <henson_sonar/coarseDM.h>
// #include <Eigen3/Eigen/Dense>


void sonarCallback(const imaging_sonar_msgs::SonarImage& msg) // TODO change this to imaging sonar message
{
  // send imaging sonar message to hensonLib functions
  // rosbag uses imaging_sonar_msgs/SonarImage messages
  printf("got a message!\n");


}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "sonarlistener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/oculus/sonar_image", 1000, sonarCallback); // TODO change first arg to name of sonar topic

  ros::spin();

  return 0;
}
