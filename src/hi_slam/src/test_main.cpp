/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-03-30
*
*/

#include<pluginlib/class_loader.h>
#include<ros/ros.h>
#include<string>
#include "slam_base.h"


int main(int argc, char** argv)
{

  ros::init(argc, argv, "test_main");

  pluginlib::ClassLoader<hi_slam::SLAMBase> slam_loader_("hi_slam", "hi_slam::SLAMBase");

  boost::shared_ptr<hi_slam::SLAMBase> laser_module_;
  try{
    laser_module_ = slam_loader_.createInstance("laser_module/Slam");
    laser_module_->Activate();
    ROS_INFO("Start slam module %s", "laser_module/Slam");
  }catch(const pluginlib::PluginlibException &ex){
    ROS_FATAL(
          "Failed to create the %s plugin, are you sure it is properly registered and that the containing library is built? Exception: %s",
          "laser_module/Slam", ex.what());
    exit(1);
  }

  boost::shared_ptr<hi_slam::SLAMBase> rgdb_module_;
  try{
    rgdb_module_ = slam_loader_.createInstance("rgbd_module/Slam");
    rgdb_module_->Activate();
    ROS_INFO("Start slam module %s", "rgbd_module/Slam");
  }catch(const pluginlib::PluginlibException &ex){
    ROS_FATAL(
          "Failed to create the %s plugin, are you sure it is properly registered and that the containing library is built? Exception: %s",
          "rgbd_module/Slam", ex.what());
    exit(1);
  }



  ros::spin();

  ros::shutdown();

  rgdb_module_->Shutdown();
  laser_module_->Shutdown();

  return 0;
}
