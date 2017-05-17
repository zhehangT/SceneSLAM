/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

constexpr int kLatestOnlyPublisherQueueSize = 1;

Node::Node(const NodeOptions& options, tf2_ros::Buffer* const tf_buffer)
    : options_(options), map_builder_bridge_(options_, tf_buffer) {}

Node::Node(const NodeOptions& options, tf2_ros::Buffer* const tf_buffer, geometry_msgs::Transform& pose, string map_frame)
    : options_(options), map_builder_bridge_(options_, tf_buffer) {

  laser_camera_transform.translation.x = 0;
  laser_camera_transform.translation.y = 0;
  laser_camera_transform.translation.z = 0;
  laser_camera_transform.rotation.x = 0;
  laser_camera_transform.rotation.y = 0;
  laser_camera_transform.rotation.z = 0;
  laser_camera_transform.rotation.w = 1;

  map_laser_transform.transform = pose;
  map_laser_transform.header.frame_id = "map";
  map_laser_transform.child_frame_id = options_.map_frame;

  path_pub = node_handle_.advertise<nav_msgs::Path>("Laser/Trajectory",10);
  path.header.frame_id = options_.map_frame;
//  path.header.frame_id = "map";
  path_count = 0;

}

Node::~Node() {
  {
    carto::common::MutexLocker lock(&mutex_);
    terminating_ = true;
  }
  if (occupancy_grid_thread_.joinable()) {
    occupancy_grid_thread_.join();
  }
}

void Node::Initialize() {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  submap_query_server_ = node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this);

  if (options_.map_builder_options.use_trajectory_builder_2d()) {
    occupancy_grid_publisher_ =
        node_handle_.advertise<::nav_msgs::OccupancyGrid>(
            kOccupancyGridTopic, kLatestOnlyPublisherQueueSize,
            true /* latched */);
    occupancy_grid_thread_ =
        std::thread(&Node::SpinOccupancyGridThreadForever, this);
  }

  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(options_.pose_publish_period_sec),
      &Node::PublishTrajectoryStates, this));
}

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }

MapBuilderBridge* Node::map_builder_bridge() { return &map_builder_bridge_; }

bool Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  return map_builder_bridge_.HandleSubmapQuery(request, response);
}

void Node::PublishSubmapList(const ::ros::WallTimerEvent& unused_timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_.publish(map_builder_bridge_.GetSubmapList());
}

void Node::PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event) {
  carto::common::MutexLocker lock(&mutex_);

  map_laser_transform.header.stamp = ros::Time::now();
  tf_broadcaster_.sendTransform(map_laser_transform);


  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    const auto& trajectory_state = entry.second;

    geometry_msgs::TransformStamped stamped_transform;
    stamped_transform.header.stamp = ToRos(trajectory_state.pose_estimate.time);

    const auto& tracking_to_local = trajectory_state.pose_estimate.pose;
    const Rigid3d tracking_to_map =
        trajectory_state.local_to_map * tracking_to_local;

    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_state.pose_estimate.time !=
        last_scan_matched_point_cloud_time_) {
      scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
          carto::common::ToUniversal(trajectory_state.pose_estimate.time),
          options_.tracking_frame,
          carto::sensor::TransformPointCloud(
              trajectory_state.pose_estimate.point_cloud,
              tracking_to_local.inverse().cast<float>())));
      last_scan_matched_point_cloud_time_ = trajectory_state.pose_estimate.time;
    } else {
      // If we do not publish a new point cloud, we still allow time of the
      // published poses to advance.
      stamped_transform.header.stamp = ros::Time::now();
    }

    if (trajectory_state.published_to_tracking != nullptr) {
      if (options_.provide_odom_frame) {
        std::vector<geometry_msgs::TransformStamped> stamped_transforms;

        stamped_transform.header.frame_id = options_.map_frame;
        // TODO(damonkohler): 'odom_frame' and 'published_frame' must be
        // per-trajectory to fully support the multi-robot use case.
        stamped_transform.child_frame_id = options_.odom_frame;
        stamped_transform.transform =
            ToGeometryMsgTransform(trajectory_state.local_to_map);
        stamped_transforms.push_back(stamped_transform);

        stamped_transform.header.frame_id = options_.odom_frame;
        stamped_transform.child_frame_id = options_.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_local * (*trajectory_state.published_to_tracking));
        stamped_transforms.push_back(stamped_transform);

        tf_broadcaster_.sendTransform(stamped_transforms);

        laser_camera_transform = ToGeometryMsgTransform(tracking_to_map);   

      } else {
        stamped_transform.header.frame_id = options_.map_frame;
        stamped_transform.child_frame_id = options_.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_map * (*trajectory_state.published_to_tracking));
        tf_broadcaster_.sendTransform(stamped_transform);

        laser_camera_transform = ToGeometryMsgTransform(tracking_to_map);


      }

      if(path_count == 0){
        geometry_msgs::PoseStamped camera_pose;
        camera_pose.pose.position.x = laser_camera_transform.translation.x;
        camera_pose.pose.position.y = laser_camera_transform.translation.y;
        camera_pose.pose.position.z = laser_camera_transform.translation.z;
        camera_pose.pose.orientation = laser_camera_transform.rotation;
        camera_pose.header.stamp = ros::Time::now();
        camera_pose.header.frame_id = options_.map_frame;
//        camera_pose.header.frame_id = "map";
        path.poses.push_back(camera_pose);
        path_pub.publish(path);
        path_count = 10;
      }

      path_count--;



    }
  }
}

void Node::SpinOccupancyGridThreadForever() {
  for (;;) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    {
      carto::common::MutexLocker lock(&mutex_);
      if (terminating_) {
        return;
      }
    }
    if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
      continue;
    }
    const auto occupancy_grid = map_builder_bridge_.BuildOccupancyGrid();
    if (occupancy_grid != nullptr) {
      occupancy_grid_publisher_.publish(*occupancy_grid);
    }
  }
}

}  // namespace cartographer_ros
