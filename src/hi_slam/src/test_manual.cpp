/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-03-23
*
*/

#include <ros/ros.h>
#include "software_bus_manual.h"


int main(int argc, char** argv)
{

  ros::init(argc, argv, "hi_slam");

  hi_slam::SoftwareBusManual softwareBus;
  softwareBus.StartSoftwareBus();

  ros::spin();

  ros::shutdown();

  return 0;
}
