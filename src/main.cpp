#include<iostream>
#include<sstream>
#include<string>
#include<vector>

#include"opencv2/opencv.hpp"
#include"FlyCap2CV.h"

int calibrate(int argc, char* argv[]){
  FlyCap2CVWrapper cam(0);
  cv::Mat frame;
  std::vector<std::vector<cv::Point2d>> corner_list;
	while(true)
  {
    std::vector<cv::Point2d> corner;
    frame = cam.readImage();
    cv::Mat gray;
    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

    found = findChessboardCorners(gray,cv::Size(8,6),corner);
    drawChessboardCorners(frame,cv::Size(8,6),corner,found);

    cv::imshow("image",frame);
    int key = cv::waitKey(1);
    if(key=='q')break;
    if(key=='s'){
      corner_list.push_back(corner);
      std::cout<<"corner saved."<<corner_list.size()<<std::endl;
    }
	}
  cv::FileStorage fs("../data/intrinsics.yml");
	return 0;
}

int main(int argc, char* argv[]){
  if(CALIBRATE){
    calibrate(argc,argv);
  }
  return 0;
}

