/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-04-09
*
*/

#include<yaml-cpp/yaml.h>
#include<iostream>
#include "config.h"



int main()
{

  hi_slam::Config config;

  YAML::Node config_yaml = YAML::LoadFile("/home/tzh/HiSLAM/src/hi_slam/data/config.yaml");
  config.SceneRecognition = config_yaml["SceneRecognition"].as<string>();
  cout << config.SceneRecognition << endl;

  YAML::Node sceneConfig_yaml = config_yaml["SceneConfig"];

  for (std::size_t i=0; i<sceneConfig_yaml.size(); i++) {

    string name = sceneConfig_yaml[i]["name"].as<string>();
    string slam = sceneConfig_yaml[i]["slam"].as<string>();
    config.SceneConfig.insert( pair<string, string>(name, slam) );

  }

  cout << config.SceneConfig["Dark"] << endl;



}
