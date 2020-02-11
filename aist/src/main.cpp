#include<iostream>
#include<sstream>
#include<string>
#include<vector>

#include"opencv2/opencv.hpp"
#include"FlyCap2CV.h"

int calibrate(int argc, char* argv[]){
  float SQ_SIZE = 0.0295;
  FlyCap2CVWrapper cam(1);
  cv::Mat frame;
  std::vector<std::vector<cv::Point2f>> corner_list;
	while(true)
  {
    std::vector<cv::Point2f> corner;
    frame = cam.readImage();
    cv::Mat gray;
    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

    bool found = cv::findChessboardCorners(gray,cv::Size(8,6),corner,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

    if(found)
      cv::cornerSubPix(gray,corner,cv::Size(11,11),cv::Size(-1,-1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT,30,0.1));

    cv::Mat draw;
    frame.copyTo(draw);
    cv::drawChessboardCorners(draw,cv::Size(8,6),cv::Mat(corner),found);

    cv::imshow("image",frame);
    cv::imshow("find",draw);
    int key = cv::waitKey(1);
    if(key == 'q')break;
    if(key == 27)break;
    if(key == 's' && found){
      corner_list.push_back(corner);
      std::cout<<"corner saved."<<corner_list.size()<<std::endl;
    }
	}
  std::vector<cv::Point3f> object;
  for(std::size_t j=0; j<cv::Size(8,6).height; j++){
    for(std::size_t i=0; i<cv::Size(8,6).width; i++){
      object.push_back(cv::Point3f(i*SQ_SIZE,j*SQ_SIZE,0.0f));
    }
  }

  std::vector<std::vector<cv::Point3f> > object_list;
  for(std::size_t i=0;i<corner_list.size();i++){
    object_list.push_back(object);
  }
  cv::Mat K,D;
  std::vector<cv::Mat> R,T;
  double RMS = cv::calibrateCamera(object_list,corner_list,cv::Size(8,6),K,D,R,T);
  
  std::cout<<"RMS"<<RMS<<std::endl;
  std::cout<<"K"<<K<<std::endl;
  std::cout<<"D"<<D<<std::endl;
  
  std::string file_name;
  std::cout<<"output file name"<<std::endl;
  std::cin >>file_name;

  cv::FileStorage fs(file_name+".yml",cv::FileStorage::WRITE);
  fs<<"RMS"<<RMS;
  fs<<"K"<<K;
  fs<<"D"<<D;
  fs.release();
	return 0;
}

int main(int argc, char* argv[]){
  if(CALIBRATE){
    calibrate(argc,argv);
  }
  return 0;
}

