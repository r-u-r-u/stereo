#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/sfm.hpp"
#define SQUARE_SIZE 0.012

int main(int argc,char* argv[]){

  cv::VideoCapture capture(1);
  cv::Mat frame;
  
  std::vector<std::vector<cv::Point2f>> corners_list;
  while(1){
    capture >> frame;
    
    cv::Mat gray;
    cv::cvtColor(frame,gray,CV_BGR2GRAY);
    
    std::vector<cv::Point2f> corners;
    bool is_found = cv::findChessboardCorners(gray,cv::Size(10,7),corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
        
    if(is_found)cv::cornerSubPix(gray,corners,cv::Size(10,7),cv::Size(-1,-1)
                  ,cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER,30,0.1));

    cv::Mat draw;
    frame.copyTo(draw);
    cv::drawChessboardCorners(draw,cv::Size(10,7),corners,is_found);
    
    cv::imshow("frame",frame);
    cv::imshow("gray",gray);
    cv::imshow("draw",draw);
    int key = cv::waitKey(1);
    if(key == 'q')break; 
    if(key == 27 )break;
    if(key == 'w' && is_found){
      corners_list.push_back(corners);
      std::cout<<"corner saved"<<corners_list.size()<<std::endl;
    }
  }
  std::vector<cv::Point3f> point_3d;
  for(std::size_t j=0;j<7;j++){
    for(std::size_t i=0;i<10;i++){
      point_3d.push_back(cv::Point3f(i*SQUARE_SIZE,j*SQUARE_SIZE,0.0f));
    }
  }
  std::vector<std::vector<cv::Point3f>> point_3d_list;
  for(std::size_t i=0;i<corners_list.size();i++){
    point_3d_list.push_back(point_3d);
  }

  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  double rms = cv::calibrateCamera(point_3d_list,corners_list,frame.size(),cameraMatrix,distCoeffs,rvecs,tvecs);
  std::cout<<"image size:"<<frame.size()<<std::endl;
  std::cout<<"RMS:"<<rms<<std::endl;
  std::cout<<"CameraMatrix:\n"<<cameraMatrix<<std::endl;
  std::cout<<"distCoeff:\n"<<distCoeffs<<std::endl;
  cv::FileStorage fs("./data/intrinsics.yml",cv::FileStorage::WRITE);
  fs<<"size"<<frame.size();
  fs<<"RMS"<<rms;
  fs<<"K"<<cameraMatrix;
  fs<<"D"<<distCoeffs;
  fs.release();
  return 0;
}
