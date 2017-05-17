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

#include "map_publisher.h"

MapPublisher::MapPublisher(ORB_SLAM2::Map* pMap, geometry_msgs::Transform& pose, string map_frame):mpMap(pMap), mbCameraUpdated(false)
{

    MAP_FRAME_ID = map_frame;
    const char* POINTS_NAMESPACE = "MapPoints";
    const char* KEYFRAMES_NAMESPACE = "KeyFrames";
    const char* GRAPH_NAMESPACE = "Graph";
    const char* CAMERA_NAMESPACE = "Camera";

    //Configure MapPoints
    fPointSize=0.01;
    mPoints.header.frame_id = MAP_FRAME_ID;
    mPoints.ns = POINTS_NAMESPACE;
    mPoints.id=0;
    mPoints.type = visualization_msgs::Marker::POINTS;
    mPoints.scale.x=fPointSize;
    mPoints.scale.y=fPointSize;
    mPoints.pose.orientation.w=1.0;
    mPoints.action=visualization_msgs::Marker::ADD;
    mPoints.color.a = 1.0;

    //Configure KeyFrames
    fCameraSize=0.04;
    mKeyFrames.header.frame_id = MAP_FRAME_ID;
    mKeyFrames.ns = KEYFRAMES_NAMESPACE;
    mKeyFrames.id=1;
    mKeyFrames.type = visualization_msgs::Marker::LINE_LIST;
    mKeyFrames.scale.x=0.005;
    mKeyFrames.pose.orientation.w=1.0;
    mKeyFrames.action=visualization_msgs::Marker::ADD;

    mKeyFrames.color.b=1.0f;
    mKeyFrames.color.a = 1.0;

    //Configure Covisibility Graph
    mCovisibilityGraph.header.frame_id = MAP_FRAME_ID;
    mCovisibilityGraph.ns = GRAPH_NAMESPACE;
    mCovisibilityGraph.id=2;
    mCovisibilityGraph.type = visualization_msgs::Marker::LINE_LIST;
    mCovisibilityGraph.scale.x=0.002;
    mCovisibilityGraph.pose.orientation.w=1.0;
    mCovisibilityGraph.action=visualization_msgs::Marker::ADD;
    mCovisibilityGraph.color.b=0.7f;
    mCovisibilityGraph.color.g=0.7f;
    mCovisibilityGraph.color.a = 0.3;

    //Configure KeyFrames Spanning Tree
    mMST.header.frame_id = MAP_FRAME_ID;
    mMST.ns = GRAPH_NAMESPACE;
    mMST.id=3;
    mMST.type = visualization_msgs::Marker::LINE_LIST;
    mMST.scale.x=0.005;
    mMST.pose.orientation.w=1.0;
    mMST.action=visualization_msgs::Marker::ADD;
    mMST.color.b=0.0f;
    mMST.color.g=1.0f;
    mMST.color.a = 1.0;

    //Configure Current Camera
    mCurrentCamera.header.frame_id = MAP_FRAME_ID;
    mCurrentCamera.ns = CAMERA_NAMESPACE;
    mCurrentCamera.id=4;
    mCurrentCamera.type = visualization_msgs::Marker::LINE_LIST;
    mCurrentCamera.scale.x=0.01;//0.2; 0.03
    mCurrentCamera.pose.orientation.w=1.0;
    mCurrentCamera.action=visualization_msgs::Marker::ADD;
    mCurrentCamera.color.g=1.0f;
    mCurrentCamera.color.a = 1.0;

    //Configure Reference MapPoints
    mReferencePoints.header.frame_id = MAP_FRAME_ID;
    mReferencePoints.ns = POINTS_NAMESPACE;
    mReferencePoints.id=6;
    mReferencePoints.type = visualization_msgs::Marker::POINTS;
    mReferencePoints.scale.x=fPointSize;
    mReferencePoints.scale.y=fPointSize;
    mReferencePoints.pose.orientation.w=1.0;
    mReferencePoints.action=visualization_msgs::Marker::ADD;
    mReferencePoints.color.r =1.0f;
    mReferencePoints.color.a = 1.0;

    //Configure Publisher
    publisher = nh.advertise<visualization_msgs::Marker>("ORB_SLAM/Map", 10);

    publisher.publish(mPoints);
    publisher.publish(mReferencePoints);
    publisher.publish(mCovisibilityGraph);
    publisher.publish(mKeyFrames);
    publisher.publish(mCurrentCamera);


    path_pub = nh.advertise<nav_msgs::Path>("ORB_SLAM/Trajectory",10);
    path.header.frame_id = MAP_FRAME_ID;
//    path.header.frame_id = "map";



    rgbd_camera_transform.translation.x = 0;
    rgbd_camera_transform.translation.y = 0;
    rgbd_camera_transform.translation.z = 0;
    rgbd_camera_transform.rotation.x = 0;
    rgbd_camera_transform.rotation.y = 0;
    rgbd_camera_transform.rotation.z = 0;
    rgbd_camera_transform.rotation.w = 1;

    map_rgbd_transform.transform = pose;
    map_rgbd_transform.header.frame_id = "map";
    map_rgbd_transform.child_frame_id = MAP_FRAME_ID;

}

void MapPublisher::Refresh()
{

  map_rgbd_transform.header.stamp = ros::Time::now();
  tf2_br.sendTransform(map_rgbd_transform);


  if(isCamUpdated())
  {
    cv::Mat Tcw = GetCurrentCameraPose();
    PublishCurrentCamera(Tcw);
    ResetCamFlag();
    SetCurrentCameraPose(Tcw);
  }

  if(mpMap->isMapUpdated())
  {
    vector<ORB_SLAM2::KeyFrame*> vKeyFrames = mpMap->GetAllKeyFrames();
    vector<ORB_SLAM2::MapPoint*> vMapPoints = mpMap->GetAllMapPoints();
    vector<ORB_SLAM2::MapPoint*> vRefMapPoints = mpMap->GetReferenceMapPoints();

    PublishMapPoints(vMapPoints, vRefMapPoints);
    PublishKeyFrames(vKeyFrames);

    mpMap->ResetUpdated();
  }
}

void MapPublisher::PublishMapPoints(const vector<ORB_SLAM2::MapPoint*> &vpMPs, const vector<ORB_SLAM2::MapPoint*> &vpRefMPs)
{
    mPoints.points.clear();
    mReferencePoints.points.clear();

    set<ORB_SLAM2::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        geometry_msgs::Point p;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        p.x=pos.at<float>(2);
//        p.y=pos.at<float>(1);
//        p.z=pos.at<float>(2);
        p.y=-pos.at<float>(0);
        p.z=pos.at<float>(1);

        mPoints.points.push_back(p);
    }

    for(set<ORB_SLAM2::MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        geometry_msgs::Point p;
        cv::Mat pos = (*sit)->GetWorldPos();
        p.x=pos.at<float>(2);
//        p.y=pos.at<float>(1);
//        p.z=pos.at<float>(2);
        p.y=-pos.at<float>(0);
        p.z=pos.at<float>(1);

        mReferencePoints.points.push_back(p);
    }

    mPoints.header.stamp = ros::Time::now();
    mReferencePoints.header.stamp = ros::Time::now();
    publisher.publish(mPoints);
    publisher.publish(mReferencePoints);
}

void MapPublisher::PublishKeyFrames(const vector<ORB_SLAM2::KeyFrame*> &vpKFs)
{
    mKeyFrames.points.clear();
    mCovisibilityGraph.points.clear();
    mMST.points.clear();

    float d = fCameraSize;

    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

    for(size_t i=0, iend=vpKFs.size() ;i<iend; i++)
    {
        cv::Mat Tcw = vpKFs[i]->GetPose();
        cv::Mat Twc = Tcw.inv();
        cv::Mat ow = vpKFs[i]->GetCameraCenter();
        cv::Mat p1w = Twc*p1;
        cv::Mat p2w = Twc*p2;
        cv::Mat p3w = Twc*p3;
        cv::Mat p4w = Twc*p4;

        geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
        msgs_o.x=ow.at<float>(2);
//        msgs_o.y=ow.at<float>(1);
//        msgs_o.z=ow.at<float>(2);
        msgs_o.y=-ow.at<float>(0);
        msgs_o.z=ow.at<float>(1);

        msgs_p1.x=p1w.at<float>(2);
//        msgs_p1.y=p1w.at<float>(1);
//        msgs_p1.z=p1w.at<float>(2);
        msgs_p1.y=-p1w.at<float>(0);
        msgs_p1.z=p1w.at<float>(1);


        msgs_p2.x=p2w.at<float>(2);
//        msgs_p2.y=p2w.at<float>(1);
//        msgs_p2.z=p2w.at<float>(2);
        msgs_p2.y=-p2w.at<float>(0);
        msgs_p2.z=p2w.at<float>(1);

        msgs_p3.x=p3w.at<float>(2);
//        msgs_p3.y=p3w.at<float>(1);
//        msgs_p3.z=p3w.at<float>(2);
        msgs_p3.y=-p3w.at<float>(0);
        msgs_p3.z=p3w.at<float>(1);

        msgs_p4.x=p4w.at<float>(2);
//        msgs_p4.y=p4w.at<float>(1);
//        msgs_p4.z=p4w.at<float>(2);
        msgs_p4.y=-p4w.at<float>(0);
        msgs_p4.z=p4w.at<float>(1);

        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p1);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p1);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p1);

        // Covisibility Graph
        vector<ORB_SLAM2::KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
        if(!vCovKFs.empty())
        {
            for(vector<ORB_SLAM2::KeyFrame*>::iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
            {
                if((*vit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Ow2 = (*vit)->GetCameraCenter();
                geometry_msgs::Point msgs_o2;
                msgs_o2.x=Ow2.at<float>(2);
//                msgs_o2.y=Ow2.at<float>(1);
//                msgs_o2.z=Ow2.at<float>(2);
                msgs_o2.y=-Ow2.at<float>(0);
                msgs_o2.z=Ow2.at<float>(1);
                mCovisibilityGraph.points.push_back(msgs_o);
                mCovisibilityGraph.points.push_back(msgs_o2);
            }
        }

        // MST
        ORB_SLAM2::KeyFrame* pParent = vpKFs[i]->GetParent();
        if(pParent)
        {
            cv::Mat Owp = pParent->GetCameraCenter();
            geometry_msgs::Point msgs_op;
            msgs_op.x=Owp.at<float>(2);
//            msgs_op.y=Owp.at<float>(1);
//            msgs_op.z=Owp.at<float>(2);
            msgs_op.y=-Owp.at<float>(0);
            msgs_op.z=Owp.at<float>(1);
            mMST.points.push_back(msgs_o);
            mMST.points.push_back(msgs_op);
        }
        set<ORB_SLAM2::KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
        for(set<ORB_SLAM2::KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
        {
            if((*sit)->mnId<vpKFs[i]->mnId)
                continue;
            cv::Mat Owl = (*sit)->GetCameraCenter();
            geometry_msgs::Point msgs_ol;
            msgs_ol.x=Owl.at<float>(2);
//            msgs_ol.y=Owl.at<float>(1);
//            msgs_ol.z=Owl.at<float>(2);
            msgs_ol.y=-Owl.at<float>(0);
            msgs_ol.z=Owl.at<float>(1);
            mMST.points.push_back(msgs_o);
            mMST.points.push_back(msgs_ol);
        }
    }

    mKeyFrames.header.stamp = ros::Time::now();
    mCovisibilityGraph.header.stamp = ros::Time::now();
    mMST.header.stamp = ros::Time::now();

//    publisher.publish(mKeyFrames);
//    publisher.publish(mCovisibilityGraph);
//    publisher.publish(mMST);
}

void MapPublisher::PublishCurrentCamera(const cv::Mat &Tcw)
{
    mCurrentCamera.points.clear();

    float d = fCameraSize;

    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

    cv::Mat Twc = Tcw.inv();
    cv::Mat ow = Twc*o;
    cv::Mat p1w = Twc*p1;
    cv::Mat p2w = Twc*p2;
    cv::Mat p3w = Twc*p3;
    cv::Mat p4w = Twc*p4;

    geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
    msgs_o.x=ow.at<float>(2);
//    msgs_o.y=ow.at<float>(1);
//    msgs_o.z=ow.at<float>(2);
    msgs_o.y=-ow.at<float>(0);
    msgs_o.z=ow.at<float>(1);

    msgs_p1.x=p1w.at<float>(2);
//    msgs_p1.y=p1w.at<float>(1);
//    msgs_p1.z=p1w.at<float>(2);
    msgs_p1.y=-p1w.at<float>(0);
    msgs_p1.z=p1w.at<float>(1);

    msgs_p2.x=p2w.at<float>(2);
//    msgs_p2.y=p2w.at<float>(1);
//    msgs_p2.z=p2w.at<float>(2);
    msgs_p2.y=-p2w.at<float>(0);
    msgs_p2.z=p2w.at<float>(1);

    msgs_p3.x=p3w.at<float>(2);
//    msgs_p3.y=p3w.at<float>(1);
//    msgs_p3.z=p3w.at<float>(2);
    msgs_p3.y=-p3w.at<float>(0);
    msgs_p3.z=p3w.at<float>(1);

    msgs_p4.x=p4w.at<float>(2);
//    msgs_p4.y=p4w.at<float>(1);
//    msgs_p4.z=p4w.at<float>(2);
    msgs_p4.y=-p4w.at<float>(0);
    msgs_p4.z=p4w.at<float>(1);

    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);

    mCurrentCamera.header.stamp = ros::Time::now();

//    publisher.publish(mCurrentCamera);


    {
    /*根据世界坐标系下相机的位姿和里程计坐标系下相机的位姿，计算相机坐标系和里程计坐标系之间的旋转矩阵和平移向量
     *
     * r_world_camera = r_world_odom * r_odom_camera
     * --> r_world_odom = r_world_camera * r_odom_camera^(-1);
     *
     * C_w = r_world_odom * C_o + t_world_odom
     * C_w = r_world_camera * [0,0,0] + t_world_camera  = t_world_camera
     * C_o = r_odom_camera * [0,0,0] + t_odom_camera  = t_odom_camera
     * --> t_world_odom = t_world_camera - r_world_odom * t_odom_camera;
     */
      tf::Matrix3x3 r_world_camera = tf::Matrix3x3(Twc.at<float>(2,2), -Twc.at<float>(2,0), Twc.at<float>(2,1),
                                                   -Twc.at<float>(0,2), Twc.at<float>(0,0), -Twc.at<float>(0,1),
                                                   Twc.at<float>(1,2), -Twc.at<float>(1,0), Twc.at<float>(1,1));
      tf::Vector3 t_world_camera(ow.at<float>(2), -ow.at<float>(0), ow.at<float>(1));


      rgbd_camera_transform.translation.x = t_world_camera.getX();
      rgbd_camera_transform.translation.y = t_world_camera.getY();
      rgbd_camera_transform.translation.z = t_world_camera.getZ();
      tf::Quaternion q;
      r_world_camera.getRotation(q);
      rgbd_camera_transform.rotation.x = q.getX();
      rgbd_camera_transform.rotation.y = q.getY();
      rgbd_camera_transform.rotation.z = q.getZ();
      rgbd_camera_transform.rotation.w = q.getW();



      tf::StampedTransform transform_odom_camera;
      try{

        listener.lookupTransform("odom", "camera_rgb_frame",
                                 ros::Time(0), transform_odom_camera);


        tf::Matrix3x3 r_odom_camera = transform_odom_camera.getBasis();
        tf::Vector3 t_odom_camera =  transform_odom_camera.getOrigin();


        tf::Matrix3x3 r_world_odom = r_world_camera * r_odom_camera.inverse();
        transform.setBasis( r_world_odom );

        tf::Vector3 t_world_odom = t_world_camera - r_world_odom * t_odom_camera;
        transform.setOrigin( t_world_odom);


        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), MAP_FRAME_ID, "odom"));

      }
      catch (tf::TransformException &ex) {

        //      ROS_ERROR("FUCK:%s",ex.what());
      }


      geometry_msgs::PoseStamped camera_pose;
      camera_pose.pose.position.x = rgbd_camera_transform.translation.x;
      camera_pose.pose.position.y = rgbd_camera_transform.translation.y;
      camera_pose.pose.position.z = rgbd_camera_transform.translation.z;
      camera_pose.pose.orientation = rgbd_camera_transform.rotation;
      camera_pose.header.stamp = ros::Time::now();
      camera_pose.header.frame_id = MAP_FRAME_ID;
//      camera_pose.header.frame_id = "map";

      path.poses.push_back(camera_pose);
      path_pub.publish(path);

    }


}

void MapPublisher::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mbCameraUpdated = true;
}

cv::Mat MapPublisher::GetCurrentCameraPose()
{
    unique_lock<mutex> lock(mMutexCamera);
//    return mCameraPose.clone();
    return mpMap->getCameraPose().clone();

}

bool MapPublisher::isCamUpdated()
{
    unique_lock<mutex> lock(mMutexCamera);
//    return mbCameraUpdated;
    return mpMap->isCamUpdated();
}

void MapPublisher::ResetCamFlag()
{
    unique_lock<mutex> lock(mMutexCamera);
//    mbCameraUpdated = false;
    mpMap->ResetCamFlag();

}


