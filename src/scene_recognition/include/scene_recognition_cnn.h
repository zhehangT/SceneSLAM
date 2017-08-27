/**
*
* Author: tzh
* Date: 2017-08-10
*
*/


#ifndef SCENE_RECOGNITION_CNN_H
#define SCENE_RECOGNITION_CNN_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "scene_recognition_base.h"
#include "classification_alexnet.h"



namespace scene_recognition {

class SceneRecognitionCNN : public hi_slam::SceneRecognitionBase {

public:

  SceneRecognitionCNN();
  ~SceneRecognitionCNN();

  void Activate();
  void Shutdown();

private:
  void GrabImage(const sensor_msgs::ImageConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::string module_path_;
  ClassifierAlexnet* classifier_;

  float last_result_[3];
  double time_;


};

}





#endif // SCENE_RECOGNITION_H