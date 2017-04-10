/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-04-06
*
*/

#ifndef SCENE_RECOGNITION_BASE_H
#define SCENE_RECOGNITION_BASE_H

#include<string>

namespace hi_slam {

/**
 * @brief The SceneRecognition class
 * all SLAM plugins should inherit this base class, and implement pluginlib-required methods.
 */
    class SceneRecognitionBase  {
    public:
      SceneRecognitionBase() {}
      virtual ~SceneRecognitionBase() {}

      virtual void Activate() {}

      virtual void Shutdown() {}

      std::string scene() {return scene_;}

    protected:
      std::string scene_;
    };

} // end namespace
#endif // SCENE_RECOGNITION_BASE_H
