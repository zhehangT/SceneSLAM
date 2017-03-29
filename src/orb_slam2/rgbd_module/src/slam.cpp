/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-03-23
*
*
*/

#include<pluginlib/class_list_macros.h>

#include<ros/ros.h>
#include<iostream>
#include<fstream>
#include<chrono>
#include "slam.h"


PLUGINLIB_EXPORT_CLASS(rgbd_module::Slam, hi_slam::SLAMBase);

namespace rgbd_module{

Slam::Slam()
  :rgb_sub_(nh_, "/camera/rgb/image_raw", 1),
   depth_sub_(nh_, "/camera/depth_registered/image_raw", 1),
   sync_(sync_pol(10), rgb_sub_,depth_sub_){

  std::string vocabulary = "/home/tzh/HiSLAM/src/orb_slam2/rgbd_module/config/ORBvoc.txt";
  std::string settings = "/home/tzh/HiSLAM/src/orb_slam2/rgbd_module/config/TUM1.yaml";
  SLAM_ = new ORB_SLAM2::System(vocabulary, settings, ORB_SLAM2::System::RGBD, false);

  igb_ = new ImageGrabber(SLAM_);

}


void Slam::Activate() {

  ROS_INFO_STREAM("rgbd_module activate...");

  sync_.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,igb_,_1,_2));

}


void Slam::Shutdown() {

  ROS_INFO_STREAM("rgbd_module shutdown...");
  SLAM_->Shutdown();
  SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

}

Slam::~Slam(){

  ROS_INFO_STREAM("rgbd_module detivate...");
  delete igb_;
  delete SLAM_;
}



}//namespace


