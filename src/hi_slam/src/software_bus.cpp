/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-03-28
*
*/


#include "software_bus.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <stdlib.h>


namespace hi_slam{

  SoftwareBus::SoftwareBus()
    :private_nh_("~"),
     slam_loader_("hi_slam", "hi_slam::SLAMBase"),
     softbus_frequency_(20),
     cfg_first_setup_(true){

    //start with "rgdb_module" will die, do not know why
    slam_module_ = slam_loader_.createInstance("laser_module");

    pose.translation.x = 0;
    pose.translation.y = 0;
    pose.translation.z = 0;
    pose.rotation.x = 0;
    pose.rotation.y = 0;
    pose.rotation.z = 0;
    pose.rotation.w = 1;

    last_pose.translation.x = 0;
    last_pose.translation.y = 0;
    last_pose.translation.z = 0;
    last_pose.rotation.x = 0;
    last_pose.rotation.y = 0;
    last_pose.rotation.z = 0;
    last_pose.rotation.w = 1;

    map_id = 0;

  }

  SoftwareBus::~SoftwareBus(){

    slam_module_->Shutdown();

  }


  void SoftwareBus::StartSoftwareBus(){

    ROS_INFO_STREAM("Initialising hi_slam ...");


    dsrv_ = new dynamic_reconfigure::Server<hi_slam::SoftwareBusConfig>(
            ros::NodeHandle("hi_slam"));
    dsrv_->getConfigDefault(default_config_);    
    ROS_INFO("Loading default scene %d", default_config_.scene);
    dynamic_reconfigure::Server<hi_slam::SoftwareBusConfig>::CallbackType cb = boost::bind(
                   &SoftwareBus::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    scene_ = default_config_.scene;
    state_ = START;
    softbusThread_ = new std::thread(&SoftwareBus::RunSoftwareBus, this);
  }

  void SoftwareBus::RunSoftwareBus(){

    ros::WallRate r(softbus_frequency_);
    ROS_INFO("Running hi_slam state machine in %d Hz", softbus_frequency_);

    while(nh_.ok()){

      switch (state_) {
      case START:
        start_slam_base_scene(scene_);
        state_ = RUNNING;
        break;
      case RUNNING:
        check_scene();
//        publish_transform();
        break;

      case RESET:
        ROS_INFO("Running hi_slam state machine in RESET");
        shutdown_slam();
        state_ = START;
        break;

      case IDLE:
        break;

      default:
        break;
      }

    r.sleep();

    }
  }


  void SoftwareBus::start_slam_base_scene(int scene){

    switch(scene_){

    case 0:

      start_slam_base_name("rgbd_module");

      break;

    case 1:

      start_slam_base_name("laser_module");
      break;

    case 2:
      break;


    }

  }

  void SoftwareBus::start_slam_base_name(std::string name){

    try{
      slam_module_ = slam_loader_.createInstance(name);
      std::string map_frame = name + "_" + std::to_string(map_id++);
      std::cout<< map_frame <<std::endl;
      slam_module_->Activate(pose, map_frame);
      ROS_INFO("Start slam module %s", name.c_str());
    }catch(const pluginlib::PluginlibException &ex){
      ROS_FATAL(
            "Failed to create the %s plugin, are you sure it is properly registered and that the containing library is built? Exception: %s",
            default_config_.slam_module.c_str(), ex.what());
      exit(1);
    }
  }


  void SoftwareBus::check_scene(){




  }




  void SoftwareBus::shutdown_slam(){

    geometry_msgs::Transform pose_temp;
    slam_module_->Shutdown(pose_temp);

//    tf2::Quaternion q1(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);
//    tf2::Quaternion q2(pose_temp.rotation.x, pose_temp.rotation.y, pose_temp.rotation.z, pose_temp.rotation.w);
    tf2::Matrix3x3 r1(tf2::Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w));
    tf2::Matrix3x3 r2(tf2::Quaternion(pose_temp.rotation.x, pose_temp.rotation.y, pose_temp.rotation.z, pose_temp.rotation.w));


    tf2::Vector3 t1(pose.translation.x, pose.translation.y, pose.translation.z);
    tf2::Vector3 t2(pose_temp.translation.x, pose_temp.translation.y, pose_temp.translation.z);
    t1 = r1 * t2 + t1;
    r1 = r1 * r2;
    tf2::Quaternion q;
    r1.getRotation(q);

//    last_pose = pose;
    pose.translation.x = t1.getX();
    pose.translation.y = t1.getY();
    pose.translation.z = t1.getZ();
    pose.rotation.x = q.getX();
    pose.rotation.y = q.getY();
    pose.rotation.z = q.getZ();
    pose.rotation.w = q.getW();



  }

//  void SoftwareBus::publish_transform(){

//    tf2_ros::StaticTransformBroadcaster tf2_br;
//    geometry_msgs::TransformStamped map_rgbd_transform;

//    map_rgbd_transform.transform = last_pose;
//    map_rgbd_transform.header.frame_id = "map";
//    map_rgbd_transform.child_frame_id = "laser_module_0";
//    map_rgbd_transform.header.stamp = ros::Time::now();
//    tf2_br.sendTransform(map_rgbd_transform);

//  }



  void SoftwareBus::reconfigureCB(hi_slam::SoftwareBusConfig &config, uint32_t level){

    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    if(cfg_first_setup_){

      ROS_INFO("Suppose default scene %d", config.scene);
      last_config_ = config;
      cfg_first_setup_ = false;
      return;
    }

    if(config.scene != last_config_.scene){

      ROS_INFO("Change to scene %d", config.scene);
      last_config_ = config;
      scene_ = config.scene;
      state_ = RESET;

    }

  }


}
