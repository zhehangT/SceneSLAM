# HiSLAM
Author: Zhehang Tong

Related Publications:

Zhehang Tong, Dianxi Shi and Shaowu Yang. **SceneSLAM: A SLAM Framework Combined with Scene Detection** IEEE International Conference on Robotics and Biomimetics (ROBIO 2017) [PDF](https://www.researchgate.net/publication/319619325_SceneSLAM_A_SLAM_Framework_Combined_with_Scene_Detection)


**1 November 2017**: Beta Version

A new scene detection module based on Alexnet to detect *Indoor*, *Outdoor* and *Bright*.

RGBD Interface is called in *Indoor* and Monocular Interface is called in *Outdoor*.


**17 May 2017**: Demo Version

HiSLAM is an extensible SLAM framework to enhance the self-adaptive performance of SLAM by combing SLAM system with scene detection. To achive the scalability, the whole framework is designed with the same interface for each module using modular structure. There are mainly two kinds of modules in our framework: scene detection module and SLAM module.  A suitable SLAM module will be called after detecting scene successfully. When scene changes, our framework can switch the SLAM module and perform coordinate transformation to get global consistent location of robot and map.


To vertify the effectiveness of our framework, we implemented a prototype system running in TurtleBot robot
equipped with Kinect sensor based on our framework. We design an scene detection module to detect *Bright* and *Dark* scene based on the color images. We design and implement two kind of SLAM module in our prototype system: RBGD SLAM module based on ORB-SLAM2 and Laser SLAM module based on Cartographer.

The system will work well no matter it is *Bright* or *Dark* in indoor environment.

![image](https://github.com/zhehangT/HiSLAM/blob/master/demo.gif)
