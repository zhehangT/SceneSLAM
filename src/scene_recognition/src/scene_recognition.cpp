/**
* This file is part of scene_recognition.
*
* Author: tzh
* Date: 2017-04-07
*
*/

#include<pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "scene_recognition.h"

PLUGINLIB_EXPORT_CLASS(scene_recognition::SceneRecognition, hi_slam::SceneRecognitionBase);

namespace scene_recognition {

SceneRecognition::SceneRecognition(){


  scene_ = "null";
  detector = cv::ORB::create();
  count_ = 2;
  ROS_INFO("Initialising SceneRecognition...");
}


SceneRecognition::~SceneRecognition(){


}


void SceneRecognition::Activate(){

  sub_ = nh_.subscribe("/camera/rgb/image_raw", 1, &SceneRecognition::GrabImage,this);



}

void SceneRecognition::Shutdown(){


}

void SceneRecognition::GrabImage(const sensor_msgs::ImageConstPtr& msg){

//  ros::WallRate r(1);

  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try
  {
    cv_ptrRGB = cv_bridge::toCvShare(msg);

    std::vector<cv::KeyPoint> keypoints;
    detector->detect(cv_ptrRGB->image, keypoints);

    cv::Mat imgShow;
    cv::drawKeypoints( cv_ptrRGB->image, keypoints, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    cv::imshow( "keypoints", imgShow );
    cv::waitKey(30);

    if(keypoints.size() < 50){
      if(count_-- <= 0){
        scene_ = "Dark";
        count_ = 20;
      }

    }
    else{
      if(count_++ >= 50){
        scene_ = "Light";
        count_ = 20;
      }
    }

//    if(count_ < 500)
//      scene_ = "Light";
//    else if(count_ >= 500 && count_ < 1000)
//      scene_ = "Dark";
//    else if(count_ >= 1000)
//      scene_ = "Light";
//    count_++;

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

//  r.sleep();
}

}//namespace
