/**
* This file is part of rgbd_module.
*
* Author: tzh
* Date: 2017-04-08
*
*
*/
#include<ros/ros.h>
#include<thread>
#include "scene_recognition.h"

class Test {

public:
  Test(scene_recognition::SceneRecognition* s)
    :sceneRecognition(s) {
  }
  ~Test(){}

  void Check();

private:
  scene_recognition::SceneRecognition* sceneRecognition;


};


void Test::Check(){


  ros::WallRate rate(1);
  while(ros::ok()){


    ROS_INFO("Get Scene:%s", sceneRecognition->scene().c_str());


    rate.sleep();
  }


}


int  main(int argc, char *argv[])
{
  ros::init(argc, argv, "SceneRecognition");
  ros::start();

  scene_recognition::SceneRecognition sceneRecognition;
  sceneRecognition.Activate();

  Test t(&sceneRecognition);
  std::thread* r =new std::thread(&Test::Check, t);


  ros::spin();
  return 0;
}


