/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-03-23
*
*/

#ifndef SLAM_BASE_H
#define SLAM_BASE_H
#include<geometry_msgs/Transform.h>
#include<string>

namespace hi_slam {

/**
 * @brief The SLAMBase class
 * all SLAM plugins should inherit this base class, and implement pluginlib-required methods.
 */
    class SLAMBase  {
    public:
        SLAMBase() {}

        virtual ~SLAMBase() {}

        virtual void Activate() {}

        virtual void Shutdown() {}

        virtual void Activate(geometry_msgs::Transform& pose, std::string map_frame) {}

        virtual void Activate(geometry_msgs::Transform& pose, std::string map_frame, std::string scene) {}

        virtual void Shutdown(geometry_msgs::Transform& pose) {}



    };

} // end namespace
#endif // SLAM_BASE_H
