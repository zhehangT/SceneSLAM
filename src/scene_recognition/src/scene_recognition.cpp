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

using namespace std;
using namespace cv;

namespace scene_recognition {

SceneRecognition::SceneRecognition(){


  scene_ = "null";
//  detector = cv::ORB::create();
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


  cv_bridge::CvImageConstPtr cv_ptrRGB;
  int ave= 0;
  int count = 1;
  try{
    cv_ptrRGB = cv_bridge::toCvShare(msg);
    cv::Mat mImGray = cv_ptrRGB->image;

    for(int y=0; y<mImGray.rows; y++){
      for(int x=0; x<mImGray.cols; x++){

        ave = ave + ((int)mImGray.at<unsigned char>(y, x) - ave) / count;
        count++;
      }
    }
//    cout << ave << " ";
    if(ave < 11){
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

  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }



}


//void SceneRecognition::GrabImage(const sensor_msgs::ImageConstPtr& msg){


//  cv_bridge::CvImageConstPtr cv_ptrRGB;
////  try
////  {
////    cv_ptrRGB = cv_bridge::toCvShare(msg);

////    std::vector<cv::KeyPoint> keypoints;
////    detector->detect(cv_ptrRGB->image, keypoints);

////    cv::Mat imgShow;
////    cv::drawKeypoints( cv_ptrRGB->image, keypoints, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
////    cv::imshow( "keypoints", imgShow );
////    cv::waitKey(30);

////    if(keypoints.size() < 50){
////      if(count_-- <= 0){
////        scene_ = "Dark";
////        count_ = 20;
////      }

////    }
////    else{
////      if(count_++ >= 50){
////        scene_ = "Light";
////        count_ = 20;
////      }
////    }
////  }
////  catch (cv_bridge::Exception& e)
////  {
////    ROS_ERROR("cv_bridge exception: %s", e.what());
////  }

//  //    if(count_ < 500)
//  //      scene_ = "Light";
//  //    else if(count_ >= 500 && count_ < 1000)
//  //      scene_ = "Dark";
//  //    else if(count_ >= 1000)
//  //      scene_ = "Light";
//  //    count_++;

//}

}//namespace
