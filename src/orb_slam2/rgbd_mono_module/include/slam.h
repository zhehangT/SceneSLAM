/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-03-23
*
*
*/

#ifndef PROJECT_SLAM_H
#define PROJECT_SLAM_H

#include<message_filters/sync_policies/approximate_time.h>
#include<message_filters/time_synchronizer.h>
#include<message_filters/subscriber.h>
#include<sensor_msgs/Image.h>
#include<ros/ros.h>
#include "image_grabber.h"
#include "slam_base.h"
#include "System.h"
#include "map_publisher.h"

namespace rgbd_mono_module{

class Slam : public hi_slam::SLAMBase {

public:

  Slam();

  void Activate();

  void Shutdown();

  void Activate(geometry_msgs::Transform& pose, string map_frame, string scene);

  void Shutdown(geometry_msgs::Transform& pose);


  ~Slam();

private:
  ORB_SLAM2::System* SLAM_;
  MapPublisher* mpMapPub_;
  ros::NodeHandle nh_;
  bool first_active_;

  ImageGrabber* igb_;
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol>* sync_;

  ros::Subscriber rgb_sub_only_;

};
}

#endif //PROJECT_SLAM_H
