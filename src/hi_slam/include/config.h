/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-04-09
*
*/

#ifndef CONFIG_H
#define CONFIG_H

#include<string>
#include<vector>
#include<map>

using namespace std;

namespace hi_slam {

struct Config{

  string SceneRecognition;
  map<string,string> SceneConfig;
};

}




#endif // CONFIG_H
