/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-03-28
*
*/


#include "software_bus.h"


namespace hi_slam{

  SoftwareBus::SoftwareBus()
    :private_nh_("~"),
     slam_loader_("hi_slam", "hi_slam::SLAMBase"),
     softbus_frequency_(20),
     cfg_first_setup_(true){

    //start with "rgdb_module" will die, do not know why
    slam_module_ = slam_loader_.createInstance("laser_module/Slam");
    StartSoftwareBus();
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
        break;

      case RESET:
        ROS_INFO("Running hi_slam state machine in RESET");
        slam_module_->Shutdown();
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

      start_slam_base_name("rgbd_module/Slam");

      break;

    case 1:

      start_slam_base_name("laser_module/Slam");
      break;

    case 2:
      break;


    }

  }

  void SoftwareBus::start_slam_base_name(std::string name){

    try{
      slam_module_ = slam_loader_.createInstance(name);
      slam_module_->Activate();
      ROS_INFO("Start slam module %s", name.c_str());
    }catch(const pluginlib::PluginlibException &ex){
      ROS_FATAL(
            "Failed to create the %s plugin, are you sure it is properly registered and that the containing library is built? Exception: %s",
            default_config_.slam_module.c_str(), ex.what());
      exit(1);
    }
  }


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
