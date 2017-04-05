/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAP_PUBLISHER_H
#define MAP_PUBLISHER_H


#include<visualization_msgs/Marker.h>
#include<mutex>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include<ros/ros.h>

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"


class MapPublisher {
public:
    MapPublisher(ORB_SLAM2::Map *pMap, geometry_msgs::Transform& pose, string map_frame);

    ORB_SLAM2::Map *mpMap;

    void Refresh();

    void PublishMapPoints(const std::vector<ORB_SLAM2::MapPoint *> &vpMPs, const std::vector<ORB_SLAM2::MapPoint *> &vpRefMPs);

    void PublishKeyFrames(const std::vector<ORB_SLAM2::KeyFrame *> &vpKFs);

    void PublishCurrentCamera(const cv::Mat &Tcw);

    void SetCurrentCameraPose(const cv::Mat &Tcw);

    geometry_msgs::Transform GetCameraPose(){return rgbd_camera_transform;}

private:

    cv::Mat GetCurrentCameraPose();

    bool isCamUpdated();

    void ResetCamFlag();

    ros::NodeHandle nh;
    ros::Publisher publisher;

    visualization_msgs::Marker mPoints;
    visualization_msgs::Marker mReferencePoints;
    visualization_msgs::Marker mKeyFrames;
    visualization_msgs::Marker mReferenceKeyFrames;
    visualization_msgs::Marker mCovisibilityGraph;
    visualization_msgs::Marker mMST;
    visualization_msgs::Marker mCurrentCamera;

    float fCameraSize;
    float fPointSize;

    cv::Mat mCameraPose;
    bool mbCameraUpdated;

    std::mutex mMutexCamera;

    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::Transform transform;

    tf2_ros::StaticTransformBroadcaster tf2_br;
    geometry_msgs::TransformStamped map_rgbd_transform;

    geometry_msgs::Transform rgbd_camera_transform;
    string MAP_FRAME_ID;

};


#endif // MAP_PUBLISHER_H
