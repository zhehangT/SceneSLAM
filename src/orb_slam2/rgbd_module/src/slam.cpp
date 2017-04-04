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
#include <stdio.h>
#include "slam.h"


PLUGINLIB_EXPORT_CLASS(rgbd_module::Slam, hi_slam::SLAMBase);

namespace rgbd_module{

Slam::Slam()
  :rgb_sub_(nh_, "/camera/rgb/image_raw", 1),
   depth_sub_(nh_, "/camera/depth_registered/image_raw", 1),
   sync_(sync_pol(10), rgb_sub_,depth_sub_){

//  char *buffer;
//  //也可以将buffer作为输出参数
//  if((buffer = getcwd(NULL, 0)) == NULL)
//  {
//    perror("getcwd error");
//  }
//  else
//  {
//    printf("fuck the cwd %s\n", buffer);
//    free(buffer);
//  }
//  当前路径为：/home/tzh/.ros

  std::string vocabulary = "/home/tzh/HiSLAM/src/orb_slam2/rgbd_module/config/ORBvoc.txt";
  std::string settings = "/home/tzh/HiSLAM/src/orb_slam2/rgbd_module/config/TUM1.yaml";
  SLAM_ = new ORB_SLAM2::System(vocabulary, settings, ORB_SLAM2::System::RGBD, false);
}


void Slam::Activate() {


  ROS_INFO_STREAM("rgbd_module activate...");

  sync_.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,igb_,_1,_2));

}

void Slam::Activate(geometry_msgs::Transform& pose){

  mpMapPub_ = new MapPublisher(SLAM_->mpMap, pose);
  igb_ = new ImageGrabber(SLAM_, mpMapPub_);


  ROS_INFO_STREAM("rgbd_module activate with pose...");

  sync_.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,igb_,_1,_2));

}

void Slam::Shutdown() {

  ROS_INFO_STREAM("rgbd_module shutdown...");
  SLAM_->Shutdown();
  SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

}


void Slam::Shutdown(geometry_msgs::Transform& pose) {

  ROS_INFO_STREAM("rgbd_module shutdown with pose...");
  pose = mpMapPub_->GetCameraPose();
  SLAM_->Shutdown();
  SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

}

Slam::~Slam(){

  ROS_INFO_STREAM("rgbd_module detivate...");
  delete igb_;
  delete SLAM_;
}



}//namespace


