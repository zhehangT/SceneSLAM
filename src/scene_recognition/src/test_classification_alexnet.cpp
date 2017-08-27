/**
* use cnn to recognize scene of Outdoor, Indoor and Dark 
*
* Author: tzh
* Date: 2017-08-12
*
*
*/

#include "classification_alexnet.h"
#include <ros/package.h>


int main(int argc, const char *argv[])
{
    ::google::InitGoogleLogging(argv[0]);

    ClassifierAlexnet* classifier;
    string module_path = ros::package::getPath("scene_recognition") + "";
    string model_file = module_path + "/caffe/deploy_fine_tuning.prototxt";
    string trained_file = module_path + "/caffe/weights.caffemodel";
    string mean_file = module_path + "/caffe/imagenet_mean.binaryproto";
    string label_file = module_path + "/caffe/scene_slam.txt";
    classifier = new ClassifierAlexnet(model_file, trained_file, mean_file, label_file);

    string imgfile = module_path + "/image/0.png"; 
    cv::Mat img = cv::imread(imgfile, -1);
    CHECK(!img.empty()) << "Unable to decode image " << imgfile;
    std::vector<Prediction> predictions = classifier->Classify(img);

    /* Print the top N predictions. */
    for (size_t i = 0; i < predictions.size(); ++i) {
        Prediction p = predictions[i];
        std::cout << std::fixed << std::setprecision(4) << p.second << " - \""
                << p.first << "\"" << std::endl;
  }     


    /* code for main function */
    return 0;
}