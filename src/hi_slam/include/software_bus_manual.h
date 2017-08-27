/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-03-28
*
*/

#ifndef SOFTWARE_BUS_H
#define SOFTWARE_BUS_H


#include<thread>
#include<string>
#include<pluginlib/class_loader.h>
#include<dynamic_reconfigure/server.h>
#include "slam_base.h"
#include "scene_recognition_base.h"
#include "hi_slam/SoftwareBusConfig.h"
#include "config.h"

namespace hi_slam {

  enum SoftwareBusState {
    START,
    RUNNING,
    RESET,
    IDLE
  };


  class SoftwareBusManual{

  public:
    SoftwareBusManual();
    ~SoftwareBusManual();

    void StartSoftwareBus();

    void RunSoftwareBus();

  private:
    /*
     * ros
     * */
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    /*
     * plugin
     * */
    pluginlib::ClassLoader<hi_slam::SLAMBase> slam_loader_;
    boost::shared_ptr<hi_slam::SLAMBase> slam_module_;



    /*
     * Software Bus Variable
     * */
    SoftwareBusState state_;
    int softbus_frequency_;
    string scene_;
    std::thread *softbusThread_;


    /*
     * dynamic_reconfigure
     * */
    dynamic_reconfigure::Server <hi_slam::SoftwareBusConfig> *dsrv_;
    void reconfigureCB(hi_slam::SoftwareBusConfig &config, uint32_t level);
    boost::recursive_mutex configuration_mutex_;
    hi_slam::SoftwareBusConfig last_config_;
    hi_slam::SoftwareBusConfig default_config_;
    bool first_setup_;
    bool if_new_module_;


    void StartSlamBaseScene(string scene);
    void StartSlamBaseName(std::string name);
    void ShutdownSlam();
    void CheckScene();
    void ReadConfig();

//    void publish_transform();

    geometry_msgs::Transform pose;
    geometry_msgs::Transform last_pose;

    int map_id;
    
    hi_slam::Config config_;


  };

}





#endif // SOFTWARE_BUS_H
