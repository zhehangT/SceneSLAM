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
#include<yaml-cpp/yaml.h>

namespace hi_slam{

  SoftwareBus::SoftwareBus()
    :private_nh_("~"),
     slam_loader_("hi_slam", "hi_slam::SLAMBase"),
     scene_recognition_loader_("hi_slam", "hi_slam::SceneRecognitionBase"),
     softbus_frequency_(1),
     first_setup_(true){

    ROS_INFO_STREAM("Initialising hi_slam ...");
    ReadConfig();
    //start with "rgdb_module" will die, do not know why
    slam_module_ = slam_loader_.createInstance("laser_module");
    scene_recognition_module_ = scene_recognition_loader_.createInstance(config_.SceneRecognition);


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

    scene_ = "";
    state_ = RUNNING;
    softbusThread_ = new std::thread(&SoftwareBus::RunSoftwareBus, this);
  }

  void SoftwareBus::RunSoftwareBus(){

    ros::WallRate r(softbus_frequency_);
    ROS_INFO("Running hi_slam state machine in %d Hz", softbus_frequency_);

    scene_recognition_module_->Activate();

    while(nh_.ok()){

      switch (state_) {
      case START:
        StartSlamBaseScene(scene_);
        state_ = RUNNING;
        break;
      case RUNNING:
        CheckScene();
        break;

      case RESET:
        ROS_INFO("Running hi_slam state machine in RESET");
        ShutdownSlam();
        state_ = START;
        break;

      case IDLE:
        break;

      default:
        break;
      }
//      ROS_INFO("Scene:%s", scene_.c_str());


    r.sleep();

    }
  }


  void SoftwareBus::StartSlamBaseScene(std::string scene){

    if(scene != ""){

      ROS_INFO("StartSlamBaseScene %s", config_.SceneConfig[scene].c_str());
      StartSlamBaseName(config_.SceneConfig[scene].c_str());
    }


  }

  void SoftwareBus::StartSlamBaseName(std::string name){

    try{
      slam_module_ = slam_loader_.createInstance(name);
      std::string map_frame = name + "_" + std::to_string(map_id++);
      slam_module_->Activate(pose, map_frame);
      ROS_INFO("Start slam module %s", name.c_str());
    }catch(const pluginlib::PluginlibException &ex){
      ROS_FATAL(
            "Failed to create the %s plugin, are you sure it is properly registered and that the containing library is built? Exception: %s",
            name.c_str(), ex.what());
      exit(1);
    }
  }


  void SoftwareBus::CheckScene(){

    std::string scene = scene_recognition_module_->scene();
    ROS_INFO("Scene: %s", scene.c_str());

    if(scene_ != scene && config_.SceneConfig.find(scene) != config_.SceneConfig.end()){
      if(first_setup_){
        scene_ = scene;
        state_ = START;
        first_setup_ = false;
      }
      else{
        scene_ = scene;
        state_ = RESET;
      }
      ROS_INFO("Change to Scene: %s", scene_.c_str());

    }

  }




  void SoftwareBus::ShutdownSlam(){

    if(scene_ != ""){
      geometry_msgs::Transform pose_temp;
      slam_module_->Shutdown(pose_temp);

      tf2::Matrix3x3 r1(tf2::Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w));
      tf2::Matrix3x3 r2(tf2::Quaternion(pose_temp.rotation.x, pose_temp.rotation.y, pose_temp.rotation.z, pose_temp.rotation.w));


      tf2::Vector3 t1(pose.translation.x, pose.translation.y, pose.translation.z);
      tf2::Vector3 t2(pose_temp.translation.x, pose_temp.translation.y, pose_temp.translation.z);
      t1 = r1 * t2 + t1;
      r1 = r1 * r2;
      tf2::Quaternion q;
      r1.getRotation(q);

      pose.translation.x = t1.getX();
      pose.translation.y = t1.getY();
      pose.translation.z = t1.getZ();
      pose.rotation.x = q.getX();
      pose.rotation.y = q.getY();
      pose.rotation.z = q.getZ();
      pose.rotation.w = q.getW();
    }
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

  void SoftwareBus::ReadConfig(){

    std::string config_path = ros::package::getPath("hi_slam") + "/data/config.yaml";

    YAML::Node config_yaml = YAML::LoadFile(config_path);
    config_.SceneRecognition = config_yaml["SceneRecognition"].as<std::string>();

    YAML::Node sceneConfig_yaml = config_yaml["SceneConfig"];

    for (std::size_t i=0; i<sceneConfig_yaml.size(); i++) {

      string name = sceneConfig_yaml[i]["name"].as<string>();
      string slam = sceneConfig_yaml[i]["slam"].as<string>();
      config_.SceneConfig.insert( pair<string, string>(name, slam) );

    }
  }


}
