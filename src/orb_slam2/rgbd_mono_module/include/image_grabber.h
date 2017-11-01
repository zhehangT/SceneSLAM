/**
*
* Author: tzh
* Date: 2017-03-28
*
*
*/

#ifndef IMAGE_GRABBER_H
#define IMAGE_GRABBER_H

#include<sensor_msgs/Image.h>
#include "map_publisher.h"
#include "System.h"

class ImageGrabber
{
public:
  ImageGrabber(ORB_SLAM2::System* pSLAM, MapPublisher* pMapPub)
     :mpSLAM(pSLAM),
      mpMapPub(pMapPub),
      change_finish_(true){}

  ~ImageGrabber(){}

  void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

  void GrabRGB(const sensor_msgs::ImageConstPtr& msgRGB);


  ORB_SLAM2::System* mpSLAM;

  MapPublisher* mpMapPub;

  bool change_finish_;
};


#endif // IMAGE_GRABBER_H
