/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-04-06
*
*/

#ifndef SCENE_RECOGNITION_H
#define SCENE_RECOGNITION_H

namespace hi_slam {

/**
 * @brief The SceneRecognition class
 * all SLAM plugins should inherit this base class, and implement pluginlib-required methods.
 */
    class SceneRecognition  {
    public:
        SceneRecognition() {}

        virtual ~SceneRecognition() {}

    };

} // end namespace
#endif // SCENE_RECOGNITION_H
