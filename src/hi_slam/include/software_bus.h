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
#include "hi_slam/SoftwareBusConfig.h"


namespace hi_slam {

  enum SoftwareBusState {
    START,
    RUNNING,
    RESET,
    IDLE
  };


  class SoftwareBus{

  public:
    SoftwareBus();
    ~SoftwareBus();

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
    int scene_;
    std::thread *softbusThread_;


    /*
     * dynamic_reconfigure
     * */
    dynamic_reconfigure::Server <hi_slam::SoftwareBusConfig> *dsrv_;
    void reconfigureCB(hi_slam::SoftwareBusConfig &config, uint32_t level);
    boost::recursive_mutex configuration_mutex_;
    hi_slam::SoftwareBusConfig last_config_;
    hi_slam::SoftwareBusConfig default_config_;
    bool cfg_first_setup_ = true;


    void start_slam_base_scene(int scene);
    void start_slam_base_name(std::string name);
    void shutdown_slam();
    void check_scene();
//    void publish_transform();

    geometry_msgs::Transform pose;
    geometry_msgs::Transform last_pose;

    int map_id;

  };

}





#endif // SOFTWARE_BUS_H
