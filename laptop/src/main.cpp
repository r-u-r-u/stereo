#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "opencv2/opencv.hpp"
#include "opencv2/sfm.hpp"
#include "opencv2/viz.hpp"
#include "opencv2/optflow.hpp"

int main(int argc,char* argv[]){

//import intrinsic parameter form yml
  cv::FileStorage fs("./data/intrinsics.yml",cv::FileStorage::READ);
  cv::Mat K,D;
  fs["K"] >> K;
  fs["D"] >> D;
  std::cout<<"intrinsics"<<std::endl;
  std::cout<<"K:"<<K<<std::endl;
  std::cout<<"D:"<<D<<std::endl;

  cv::VideoCapture capture(1);
  
  cv::Mat frame_before;
  capture >> frame_before;
  
  std::cout<<"image channels:"<<frame_before.channels()<<std::endl;
  
  while(true){
//capture raw frame;
    cv::Mat  frame,remap;
    capture >> frame;
    
    cv::undistort(frame,remap,K,D);
    
    
        
    imshow("frame",frame);
    imshow("remap",remap);
    int key = cv::waitKey(1);
    if(key=='q'){
      break;
    }
  }
  return 0;
}
