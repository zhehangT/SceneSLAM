/**
* This file is part of hi_slam.
*
* Author: tzh
* Date: 2017-03-23
*
*/

#ifndef SLAM_BASE_H
#define SLAM_BASE_H


namespace hi_slam {

/**
 * @brief The SLAMBase class
 * all SLAM plugins should inherit this base class, and implement pluginlib-required methods.
 */
    class SLAMBase  {
    public:
        SLAMBase() {}

        virtual ~SLAMBase() {}

        virtual void Activate() = 0;

        virtual void Shutdown() = 0;

    };

} // end namespace
#endif // SLAM_BASE_H
