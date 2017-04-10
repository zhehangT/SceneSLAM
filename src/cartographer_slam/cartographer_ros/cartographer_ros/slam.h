/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-03-27
*
*
*/

#ifndef SLAM_H
#define SLAM_H
#include<ros/ros.h>
#include<string>
#include<vector>
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer_ros/ros_log_sink.h"
#include "tf2_ros/transform_listener.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node.h"
#include "gflags/gflags.h"
#include "slam_base.h"


namespace cartographer_ros {

class Slam : public hi_slam::SLAMBase {
public:

  Slam();

  ~Slam();

  void Activate();

  void Activate(geometry_msgs::Transform& pose, string map_frame);

  void Shutdown();

  void Shutdown(geometry_msgs::Transform& pose);

private:

  NodeOptions LoadOptions();
  void Run();

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf;
  Node* node;
  NodeOptions options_;

  int trajectory_id;
  std::unordered_set<string> expected_sensor_ids;

  ros::Subscriber laser_scan_subscriber;
  std::vector<ros::Subscriber> point_cloud_subscribers;
  ros::Subscriber imu_subscriber;
  ros::Subscriber odometry_subscriber;
  ros::ServiceServer finish_trajectory_server;

};
}//namespace


#endif // SLAM_H
