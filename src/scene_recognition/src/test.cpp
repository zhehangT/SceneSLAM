/**
* This file is part of rgbd_module.
*
* Author: tzh
* Date: 2017-05-10
*
*
*/

#include <opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>


using namespace std;
using namespace cv;



void LoadImages(const string &strFile, vector<string> &vstrImageFilenames)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            string sRGB;
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}

int  main(int argc, char *argv[])
{
  vector<string> vstrImageFilenames;
  string strPath = "/home/tzh/Data/image/";
  LoadImages(strPath + "rgb.txt" , vstrImageFilenames);

  int nImages = vstrImageFilenames.size();
  cv::Mat mImGray;
  double t;
  t = (double)getTickCount();
  for(int ni=0; ni < nImages; ni++){

    cout << strPath + vstrImageFilenames[ni] << endl;
    mImGray = cv::imread(strPath + vstrImageFilenames[ni], IMREAD_GRAYSCALE);

    float ave = 0;
    int count = 1;
    float num = 0;
    for(int y=0; y<mImGray.rows; y++){
      for(int x=0; x<mImGray.cols; x++){

        if((int)mImGray.at<unsigned char>(y, x) > 100){

          num++;
        }

        ave = ave + ((int)mImGray.at<unsigned char>(y, x) - ave) / count;
        count++;


      }
    }

    cout << ave << " " << num / (640.0 * 480) << endl;




//    imshow( "Display window", mImGray );
//    waitKey(0);

  }

  t = 1000*((double)getTickCount() - t)/getTickFrequency();
  t /= nImages;

  cout << "t: "<< t << endl;



  return 0;
}
