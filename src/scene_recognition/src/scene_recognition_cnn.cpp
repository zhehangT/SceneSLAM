/**
* use cnn to recognize scene of Outdoor, Indoor and Dark 
*
* Author: tzh
* Date: 2017-08-10
*
*
*/

#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include "scene_recognition_cnn.h"


PLUGINLIB_EXPORT_CLASS(scene_recognition::SceneRecognitionCNN, hi_slam::SceneRecognitionBase);

using namespace std;
using namespace cv;

namespace scene_recognition {

SceneRecognitionCNN::SceneRecognitionCNN(){

  module_path_ = ros::package::getPath("scene_recognition") + "/caffe/";
  string model_file = module_path_ + "deploy_fine_tuning.prototxt";
  string trained_file = module_path_ + "weights.caffemodel";
  string mean_file = module_path_ + "imagenet_mean.binaryproto";
  string label_file = module_path_ + "scene_slam.txt";
  classifier_ = new ClassifierAlexnet(model_file, trained_file, mean_file, label_file);

  for (size_t i = 0; i < 3; ++i) {
    last_result_[i] = 1;
  }
  time_ = ros::Time::now().toSec();

  ROS_INFO("Initialising SceneRecognition...");
}


SceneRecognitionCNN::~SceneRecognitionCNN(){


}


void SceneRecognitionCNN::Activate(){

  sub_ = nh_.subscribe("/camera/rgb/image_raw", 1, &SceneRecognitionCNN::GrabImage, this);



}

void SceneRecognitionCNN::Shutdown(){


}

void SceneRecognitionCNN::GrabImage(const sensor_msgs::ImageConstPtr& msg){

  cv_bridge::CvImageConstPtr cv_ptrRGB;

  if(ros::Time::now().toSec() - time_ > 1){
    try{

      // double t = (double)getTickCount();
      // double start =ros::Time::now().toSec();


      cv_ptrRGB = cv_bridge::toCvShare(msg);
      cv::Mat mImGray = cv_ptrRGB->image;

      std::vector<Prediction> predictions = classifier_->Classify(mImGray);

      // for (size_t i = 0; i < predictions.size(); ++i) {
      //     if(p.second < predictions[i].second){
      //       p = predictions[i];
      //     }
      // }
      float temp_sum = 0;
      for (size_t i = 0; i < predictions.size(); ++i){

        predictions[i].second *= last_result_[i];
        temp_sum += predictions[i].second;

      }

      Prediction p = predictions[0];
      for (size_t i = 0; i < predictions.size(); ++i) {
          if(p.second < predictions[i].second){
            p = predictions[i];
          }
          last_result_[i] = predictions[i].second / temp_sum;
      }



      scene_ = p.first;
      // double t = ros::Time::now().toSec() - start;
      // t = ((double)getTickCount() - t)/getTickFrequency();
      // ROS_INFO("%f", 1000 * t);     
      // cout << "Scene:" << p.first << "Prob:" << p.second <<endl;
      time_ = ros::Time::now().toSec();

    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

  }



}


}
