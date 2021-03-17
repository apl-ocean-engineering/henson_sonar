#include "image_proc/sonarImageProc.h"
#include <opencv2/imgproc.hpp>

//TODO ..?
const float ThetaShift = 1.5*3.14;

SonarImageProc::SonarImageProc(){

}

void SonarImageProc::parseImg(const imaging_sonar_msgs::SonarImage::ConstPtr& msg){
    //TODO code from imaging sonar processing... overkill...?
    std::vector<uint8_t> sonar_intensities = msg->intensities;
    auto sonar_ranges = msg->ranges;
    int range_length = sonar_ranges.size();
    auto sonar_bearings = msg->azimuth_angles;
    int bearing_length = sonar_bearings.size();

    cv::Mat polar, cartesian;
    polar.create(cv::Size(range_length,bearing_length), CV_8UC1);
    polar.setTo( 0 );
    
    cartesian.create(cv::Size(range_length,bearing_length), CV_8UC1);
    cartesian.setTo( 0 );

    float maxRange = 2; //TODO hardcode
    float rangeMin = 0.1;

    const float rangeMax = maxRange; //( maxRange > 0.0 ? maxRange : ping.maxRange() );

    const float rangeRes = ( rangeMax - rangeMax ) / range_length;

    const int nEffectiveRanges = ceil(rangeMax / rangeRes);

    const unsigned int radius = polar.size().height;
    const cv::Point origin(polar.size().width/2, polar.size().height);

    const float binThickness = 2 * ceil(radius / nEffectiveRanges);

    struct BearingEntry {
        float begin, center, end;

        BearingEntry( float b, float c, float e )
            : begin( b ), center(c), end(e)
            {;}
    };

    std::vector<BearingEntry> angles;
    angles.reserve( bearing_length );

    for ( int b = 0; b < bearing_length; ++b ) {
        const float center = sonar_bearings.at(b);
        float begin = 0.0, end = 0.0;

        if (b == 0) {
        end = (sonar_bearings.at(b + 1) + center) / 2.0;
        begin = 2 * center - end;

        } else if (b == bearing_length - 1) {
        begin = angles[b - 1].end;
        end = 2 * center - begin;

        } else {

        begin = angles[b - 1].end;
        end = (sonar_bearings.at(b + 1) + center) / 2.0;
        }

        angles.push_back( BearingEntry(begin, center, end) );
    }
    int count=0;
    for ( int r = 0; r < range_length; ++r ) {
        for ( int b = 0; b < bearing_length; ++b, ++count ) {

        const float range = sonar_ranges.at(r);
        int intensity = sonar_intensities.at(count);

        // QUESTION: Why are we rotating here?
        const float begin = angles[b].begin + ThetaShift,
                    end = angles[b].end + ThetaShift;

        // std::cout << begin * 180/3.14 << std::endl;

        const float rad = float(radius) * range/rangeMax;

        cartesian.at<uchar>(b,r) = intensity;

        // Assume angles are in image frame x-right, y-down
        cv::ellipse(polar, origin, cv::Size(rad, rad), 0,
                    begin * 180/3.14, end * 180/3.14,
                    intensity,
                    binThickness);
        }
    }
    _polarImage = polar;
    _cartesianImage = cartesian;
}

cv::Mat SonarImageProc::polarImage(){
    return _polarImage;
}

cv::Mat SonarImageProc::cartesianImage(){
    return _cartesianImage;
}