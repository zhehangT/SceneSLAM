/**
* This file is part of rgbd_module.
*
* Author: tzh
* Date: 2017-05-10
*
*
*/


#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>

using namespace std;

vector<float> aves;

void GrabImage(const sensor_msgs::ImageConstPtr& msg){

  cv_bridge::CvImageConstPtr cv_ptrRGB;
  float ave= 0;
  int count = 1;
  float num = 0;
  float total = 640 * 480;
  try{
    cv_ptrRGB = cv_bridge::toCvShare(msg);
    cv::Mat mImGray = cv_ptrRGB->image;

    for(int y=0; y<mImGray.rows; y++){
      for(int x=0; x<mImGray.cols; x++){

        if((int)mImGray.at<unsigned char>(y, x) > 110){
          num++;
        }

        ave = ave + ((int)mImGray.at<unsigned char>(y, x) - ave) / count;
        count++;
      }
    }
    cout << ave << endl;
    aves.push_back(ave);

  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

void WriteAves(){

    ofstream out("/home/tzh/Data/Aves.txt");

    if (out.is_open())
    {
      for(int i=0; i<aves.size(); i++){

        out << aves[i] << endl;

      }
      out.close();
    }


}



int  main(int argc, char *argv[])
{

  ros::init(argc, argv, "SceneRecognition");
  ros::start();
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/camera/rgb/image_raw", 1, &GrabImage);
  ros::spin();

  WriteAves();

  return 0;

}
