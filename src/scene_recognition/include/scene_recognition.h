/**
* This file is part of scene_recognition.
*
* Author: tzh
* Date: 2017-04-07
*
*/

#ifndef SCENE_RECOGNITION_H
#define SCENE_RECOGNITION_H

#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include <opencv2/features2d/features2d.hpp>
#include "scene_recognition_base.h"



namespace scene_recognition {

class SceneRecognition : public hi_slam::SceneRecognitionBase {

public:

  SceneRecognition();
  ~SceneRecognition();

  void Activate();
  void Shutdown();

private:
  void GrabImage(const sensor_msgs::ImageConstPtr& msg);


  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  cv::Ptr<cv::ORB> detector;

  int count_;

};

}





#endif // SCENE_RECOGNITION_H
