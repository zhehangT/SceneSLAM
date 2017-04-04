/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-03-28
*
*
*/


#include <pluginlib/class_list_macros.h>
#include "slam.h"

PLUGINLIB_EXPORT_CLASS(cartographer_ros::Slam, hi_slam::SLAMBase);

namespace cartographer_ros {

Slam::Slam()
  :tf_buffer(ros::Duration(1e6)),
   tf(tf_buffer){

}

Slam::~Slam(){

  ROS_INFO_STREAM("laser_module detivate...");
}


void Slam::Activate(){

  ROS_INFO_STREAM("laser_module activate...");

  google::InitGoogleLogging("laser_module");

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  run();
}


void Slam::Activate(geometry_msgs::Transform& pose){

  node = new Node(loadOptions(), &tf_buffer, pose);

  ROS_INFO_STREAM("laser_module activate with pose...");

  google::InitGoogleLogging("laser_module");

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  run();
}


void Slam::Shutdown(geometry_msgs::Transform& pose) {

  ROS_INFO_STREAM("laser_module shutdown...");

  node->map_builder_bridge()->FinishTrajectory(trajectory_id);
  google::ShutdownGoogleLogging();
  pose = node->current_pose;
  delete node;
}


void Slam::Shutdown() {

  ROS_INFO_STREAM("laser_module shutdown...");
  node->map_builder_bridge()->FinishTrajectory(trajectory_id);
  google::ShutdownGoogleLogging();
  delete node;
}


NodeOptions Slam::loadOptions(){
  auto file_resolver = cartographer::common::make_unique<
      cartographer::common::ConfigurationFileResolver>(
      std::vector<string>{"/home/tzh/HiSLAM/src/cartographer_slam/cartographer_turtlebot/cartographer_turtlebot/configuration_files"});
  const string code =
      file_resolver->GetFileContentOrDie("turtlebot_depth_camera_2d.lua");
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return CreateNodeOptions(&lua_parameter_dictionary);

}

void Slam::run(){

  constexpr int kInfiniteSubscriberQueueSize = 0;

  options = loadOptions();

  node->Initialize();

  trajectory_id = -1;

  // For 2D SLAM, subscribe to exactly one horizontal laser.
  if (options.use_laser_scan) {
    laser_scan_subscriber = node->node_handle()->subscribe(
        kLaserScanTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)>(
            [&](const sensor_msgs::LaserScan::ConstPtr& msg) {
              node->map_builder_bridge()
                  ->sensor_bridge(trajectory_id)
                  ->HandleLaserScanMessage(kLaserScanTopic, msg);
            }));
    expected_sensor_ids.insert(kLaserScanTopic);
  }
  if (options.use_multi_echo_laser_scan) {
    laser_scan_subscriber = node->node_handle()->subscribe(
        kMultiEchoLaserScanTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::MultiEchoLaserScan::ConstPtr&)>(
            [&](const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
              node->map_builder_bridge()
                  ->sensor_bridge(trajectory_id)
                  ->HandleMultiEchoLaserScanMessage(kMultiEchoLaserScanTopic,
                                                    msg);
            }));
    expected_sensor_ids.insert(kMultiEchoLaserScanTopic);
  }

  // For 3D SLAM, subscribe to all point clouds topics.

  if (options.num_point_clouds > 0) {
    for (int i = 0; i < options.num_point_clouds; ++i) {
      string topic = kPointCloud2Topic;
      if (options.num_point_clouds > 1) {
        topic += "_" + std::to_string(i + 1);
      }
      point_cloud_subscribers.push_back(node->node_handle()->subscribe(
          topic, kInfiniteSubscriberQueueSize,
          boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>(
              [&, topic](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                node->map_builder_bridge()
                    ->sensor_bridge(trajectory_id)
                    ->HandlePointCloud2Message(topic, msg);
              })));
      expected_sensor_ids.insert(topic);
    }
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.

  if (options.map_builder_options.use_trajectory_builder_3d() ||
      (options.map_builder_options.use_trajectory_builder_2d() &&
       options.map_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    imu_subscriber = node->node_handle()->subscribe(
        kImuTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::Imu::ConstPtr& msg)>(
            [&](const sensor_msgs::Imu::ConstPtr& msg) {
              node->map_builder_bridge()
                  ->sensor_bridge(trajectory_id)
                  ->HandleImuMessage(kImuTopic, msg);
            }));
    expected_sensor_ids.insert(kImuTopic);
  }

  // For both 2D and 3D SLAM, odometry is optional.

  if (options.use_odometry) {
    odometry_subscriber = node->node_handle()->subscribe(
        kOdometryTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const nav_msgs::Odometry::ConstPtr&)>(
            [&](const nav_msgs::Odometry::ConstPtr& msg) {
              node->map_builder_bridge()
                  ->sensor_bridge(trajectory_id)
                  ->HandleOdometryMessage(kOdometryTopic, msg);
            }));
    expected_sensor_ids.insert(kOdometryTopic);
  }

  trajectory_id = node->map_builder_bridge()->AddTrajectory(
      expected_sensor_ids, options.tracking_frame);

  finish_trajectory_server =
      node->node_handle()->advertiseService(
          kFinishTrajectoryServiceName,
          boost::function<bool(
              ::cartographer_ros_msgs::FinishTrajectory::Request&,
              ::cartographer_ros_msgs::FinishTrajectory::Response&)>([&](
              ::cartographer_ros_msgs::FinishTrajectory::Request& request,
              ::cartographer_ros_msgs::FinishTrajectory::Response&) {
            const int previous_trajectory_id = trajectory_id;
            trajectory_id = node->map_builder_bridge()->AddTrajectory(
                expected_sensor_ids, options.tracking_frame);
            node->map_builder_bridge()->FinishTrajectory(previous_trajectory_id);
            node->map_builder_bridge()->WriteAssets(request.stem);
            return true;
          }));
}




//void Slam::run(){

//  constexpr int kInfiniteSubscriberQueueSize = 0;

//  options = loadOptions();

//  node.Initialize();

//  trajectory_id = -1;

//  // For 2D SLAM, subscribe to exactly one horizontal laser.
//  if (options.use_laser_scan) {
//    laser_scan_subscriber = node.node_handle()->subscribe(
//        kLaserScanTopic, kInfiniteSubscriberQueueSize,
//        boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)>(
//            [&](const sensor_msgs::LaserScan::ConstPtr& msg) {
//              node.map_builder_bridge()
//                  ->sensor_bridge(trajectory_id)
//                  ->HandleLaserScanMessage(kLaserScanTopic, msg);
//            }));
//    expected_sensor_ids.insert(kLaserScanTopic);
//  }
//  if (options.use_multi_echo_laser_scan) {
//    laser_scan_subscriber = node.node_handle()->subscribe(
//        kMultiEchoLaserScanTopic, kInfiniteSubscriberQueueSize,
//        boost::function<void(const sensor_msgs::MultiEchoLaserScan::ConstPtr&)>(
//            [&](const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
//              node.map_builder_bridge()
//                  ->sensor_bridge(trajectory_id)
//                  ->HandleMultiEchoLaserScanMessage(kMultiEchoLaserScanTopic,
//                                                    msg);
//            }));
//    expected_sensor_ids.insert(kMultiEchoLaserScanTopic);
//  }

//  // For 3D SLAM, subscribe to all point clouds topics.

//  if (options.num_point_clouds > 0) {
//    for (int i = 0; i < options.num_point_clouds; ++i) {
//      string topic = kPointCloud2Topic;
//      if (options.num_point_clouds > 1) {
//        topic += "_" + std::to_string(i + 1);
//      }
//      point_cloud_subscribers.push_back(node.node_handle()->subscribe(
//          topic, kInfiniteSubscriberQueueSize,
//          boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>(
//              [&, topic](const sensor_msgs::PointCloud2::ConstPtr& msg) {
//                node.map_builder_bridge()
//                    ->sensor_bridge(trajectory_id)
//                    ->HandlePointCloud2Message(topic, msg);
//              })));
//      expected_sensor_ids.insert(topic);
//    }
//  }

//  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
//  // required.

//  if (options.map_builder_options.use_trajectory_builder_3d() ||
//      (options.map_builder_options.use_trajectory_builder_2d() &&
//       options.map_builder_options.trajectory_builder_2d_options()
//           .use_imu_data())) {
//    imu_subscriber = node.node_handle()->subscribe(
//        kImuTopic, kInfiniteSubscriberQueueSize,
//        boost::function<void(const sensor_msgs::Imu::ConstPtr& msg)>(
//            [&](const sensor_msgs::Imu::ConstPtr& msg) {
//              node.map_builder_bridge()
//                  ->sensor_bridge(trajectory_id)
//                  ->HandleImuMessage(kImuTopic, msg);
//            }));
//    expected_sensor_ids.insert(kImuTopic);
//  }

//  // For both 2D and 3D SLAM, odometry is optional.

//  if (options.use_odometry) {
//    odometry_subscriber = node.node_handle()->subscribe(
//        kOdometryTopic, kInfiniteSubscriberQueueSize,
//        boost::function<void(const nav_msgs::Odometry::ConstPtr&)>(
//            [&](const nav_msgs::Odometry::ConstPtr& msg) {
//              node.map_builder_bridge()
//                  ->sensor_bridge(trajectory_id)
//                  ->HandleOdometryMessage(kOdometryTopic, msg);
//            }));
//    expected_sensor_ids.insert(kOdometryTopic);
//  }

//  trajectory_id = node.map_builder_bridge()->AddTrajectory(
//      expected_sensor_ids, options.tracking_frame);

//  finish_trajectory_server =
//      node.node_handle()->advertiseService(
//          kFinishTrajectoryServiceName,
//          boost::function<bool(
//              ::cartographer_ros_msgs::FinishTrajectory::Request&,
//              ::cartographer_ros_msgs::FinishTrajectory::Response&)>([&](
//              ::cartographer_ros_msgs::FinishTrajectory::Request& request,
//              ::cartographer_ros_msgs::FinishTrajectory::Response&) {
//            const int previous_trajectory_id = trajectory_id;
//            trajectory_id = node.map_builder_bridge()->AddTrajectory(
//                expected_sensor_ids, options.tracking_frame);
//            node.map_builder_bridge()->FinishTrajectory(previous_trajectory_id);
//            node.map_builder_bridge()->WriteAssets(request.stem);
//            return true;
//          }));
//}


}//namespace
