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
  
//  cv::Ptr<cv::DenseOpticalFlow> opt = cv::optflow::createOptFlow_DeepFlow();
//  cv::Ptr<cv::DenseOpticalFlow> opt = cv::optflow::createOptFlow_Farneback();
  cv::Ptr<cv::DenseOpticalFlow> opt = cv::optflow::createOptFlow_PCAFlow();

  while(true){
//capture raw frame;
    cv::Mat  frame,flow;
    capture >> frame;
    
    cv::Mat gray_f,gray_b;
    cv::cvtColor(frame,gray_f,cv::COLOR_BGR2GRAY);
    cv::cvtColor(frame_before,gray_b,cv::COLOR_BGR2GRAY);
    opt->calc(gray_f,gray_b,flow);    
//    cv::optflow::calcOpticalFlowSF(frame_before,frame,flow,frame.channels(),2,4);

 //描画用の変換
    std::vector<cv::Mat> vflow;
    cv::split(flow, vflow);

    cv::Mat magnitude, angle;
    cv::cartToPolar(vflow[0], vflow[1], magnitude, angle, true);

    cv::Mat hsvPlanes[3];
    hsvPlanes[0] = angle;
    normalize(magnitude, magnitude, 0.0, 1.0, cv::NORM_MINMAX); // 正規化
    hsvPlanes[1] = magnitude;
    hsvPlanes[2] = cv::Mat::ones(magnitude.size(), CV_32F);

 //HSVを合成して一枚の画像にする
    cv::Mat hsv;
    cv::merge(hsvPlanes, 3, hsv);

//HSVからBGRに変換
    cv::Mat buf,flowBgr;
    cv::cvtColor(hsv, buf, cv::COLOR_HSV2BGR);
    buf *= 255.0;
    buf.convertTo(flowBgr, CV_8UC3);

    imshow("frame",frame);
    imshow("flow",flowBgr);
    int key = cv::waitKey(1);
    if(key=='q'){
      break;
    }

//copy old frame to before
    frame.copyTo(frame_before);
  }
  return 0;
}
