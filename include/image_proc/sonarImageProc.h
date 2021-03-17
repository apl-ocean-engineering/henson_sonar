//Helper class to de-code sonar image to range-bearing format and arch format
#include <opencv2/core.hpp>
#include "imaging_sonar_msgs/SonarImage.h"

class SonarImageProc{
public:
    SonarImageProc();
    void parseImg(const imaging_sonar_msgs::SonarImage::ConstPtr& msg);
    cv::Mat cartesianImage();
    cv::Mat polarImage();
private:
cv::Mat _cartesianImage;
cv::Mat _polarImage;

};