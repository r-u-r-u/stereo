#include<iostream>
#include<sstream>
#include<string>
#include<vector>

#include"opencv2/opencv.hpp"
#include"opencv2/cudastereo.hpp"
#include"FlyCap2CV.h"

int stereo_match(int argc,char* argv[]){
  FlyCap2CVWrapper cam0(0);
  FlyCap2CVWrapper cam1(1);
  std::cout << cam0.getCameraSN()<<std::endl;
  std::cout << cam1.getCameraSN()<<std::endl;
  cv::FileStorage fs;
  fs.open("../data/stereo_extrinsics.yml",cv::FileStorage::READ);
  cv::Mat K0,D0,K1,D1,R,T,E,F;
  fs["K0"] >> K0;
  fs["D0"] >> D0;
  fs["K1"] >> K1;
  fs["D1"] >> D1;
  fs["R"] >> R;
  fs["T"] >> T;
  fs["E"] >> E;
  fs["F"] >> F;
  std::cout <<"K0"<< K0 <<std::endl;
  std::cout <<"K1"<< K1 <<std::endl;
  std::cout <<"D0"<< D0 <<std::endl;
  std::cout <<"D1"<< D1 <<std::endl;
  std::cout <<"E"<< E <<std::endl;
  std::cout <<"F"<< F <<std::endl;
  std::cout <<"R"<< R <<std::endl;
  std::cout <<"T"<< T <<std::endl;

  cv::Mat frame0,frame1;
  cv::Mat disp;
  cv::Mat gray0,gray1;
  std::vector<cv::Point2f> corner0,corner1;
  std::vector<std::vector<cv::Point2f>> corner0_list,corner1_list;
    
  frame0 = cam0.readImage();
  frame1 = cam1.readImage();
  cv::cvtColor(frame0,gray0,cv::COLOR_BGR2GRAY);
  cv::cvtColor(frame1,gray1,cv::COLOR_BGR2GRAY);
  cv::Mat combine(cv::Size(frame0.cols*2,frame0.rows),CV_8UC3);

  cv::Mat R0,R1,P0,P1,Q;
  cv::stereoRectify(K0,D0,K1,D1,frame0.size(),R,T,R0,R1,P0,P1,Q);
  cv::Mat map01,map02,map11,map12;
  cv::initUndistortRectifyMap(K0,D0,R,K0,frame0.size(),CV_32FC1,map01,map02);
  cv::initUndistortRectifyMap(K1,D1,R,K1,frame1.size(),CV_32FC1,map11,map12);
//  cv::Ptr<cv::cuda::StereoBM> sm = cv::cuda::createStereoBM(256,3);
  cv::Ptr<cv::cuda::DisparityBilateralFilter> dbf = cv::cuda::createDisparityBilateralFilter(256,3,1);
//  cv::Ptr<cv::cuda::StereoBeliefPropagation> sm = cv::cuda::createStereoBeliefPropagation(256);
  cv::Ptr<cv::cuda::StereoConstantSpaceBP> sm = cv::cuda::createStereoConstantSpaceBP(256);


  cv::VideoWriter writer("../data/disparity_video.avi",cv::VideoWriter::fourcc('D', 'I', 'V', '3'),30.0,frame0.size());
  while(1){
    frame0 = cam0.readImage();
    frame1 = cam1.readImage();
    cv::remap(frame0,frame0,map01,map02,cv::INTER_LINEAR);
    cv::remap(frame1,frame1,map11,map12,cv::INTER_LINEAR);
    cv::cvtColor(frame0,gray0,cv::COLOR_BGR2GRAY);
    cv::cvtColor(frame1,gray1,cv::COLOR_BGR2GRAY);

    cv::cuda::GpuMat d_left,d_right;
    d_left.upload(gray0);
    d_right.upload(gray1);
    cv::cuda::GpuMat d_disp(frame0.size(), CV_8UC1);

    sm->compute(d_left,d_right,d_disp);
    dbf->apply(d_disp,d_left,d_disp);
    d_disp.download(disp);
     
//    cv::Mat imageLeft(combine, cv::Rect(0, 0, frame0.cols, frame0.rows));
//    cv::Mat imageRight(combine, cv::Rect(frame0.cols, 0, frame1.cols, frame1.rows));
//    frame0.copyTo(imageLeft);
//    frame1.copyTo(imageRight);
//    cv::imshow("combined",combine);

    disp.convertTo(disp,CV_8U,255.0/(256*16.0));
    cv::imshow("disp",disp);
    int key = cv::waitKey(1);
    if(key == 'q')break;
    if(key == 27)break;
    cv::cvtColor(disp,disp,cv::COLOR_GRAY2BGR);
    writer << disp;
    //d_writer->write(d_disp);
  }
  return 0;
}

int stereo_calibrate(int argc,char* argv[]){
  float SQ_SIZE = 0.0295;
  FlyCap2CVWrapper cam0(0);
  FlyCap2CVWrapper cam1(1);
  std::cout << cam0.getCameraSN()<<std::endl;
  std::cout << cam1.getCameraSN()<<std::endl;
  cv::FileStorage fs0;
  cv::FileStorage fs1;
  if(cam0.getCameraSN()=="12380091"){
    fs0.open("../data/left_intrinsics.yml",cv::FileStorage::READ);
    fs1.open("../data/right_intrinsics.yml",cv::FileStorage::READ);
  }else{
    fs1.open("../data/left_intrinsics.yml",cv::FileStorage::READ);
    fs0.open("../data/right_intrinsics.yml",cv::FileStorage::READ);
  }
  cv::Mat K0,D0,K1,D1;
  fs0["K"] >> K0;
  fs0["D"] >> D0;
  fs1["K"] >> K1;
  fs1["D"] >> D1;
  std::cout << K0 <<std::endl;
  std::cout << K1 <<std::endl;
  std::cout << D0 <<std::endl;
  std::cout << D1 <<std::endl;

  cv::Mat frame0,frame1;
  cv::Mat gray0,gray1;
  std::vector<cv::Point2f> corner0,corner1;
  std::vector<std::vector<cv::Point2f>> corner0_list,corner1_list;
  while(1){
    frame0 = cam0.readImage();
    frame1 = cam1.readImage();
    cvtColor(frame0,gray0,cv::COLOR_BGR2GRAY);
    cvtColor(frame1,gray1,cv::COLOR_BGR2GRAY);

    bool found0 = cv::findChessboardCorners(gray0,cv::Size(8,6),corner0,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
    bool found1 = cv::findChessboardCorners(gray1,cv::Size(8,6),corner1,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

    if(found0){
      cv::cornerSubPix(gray0,corner0,cv::Size(11,11),cv::Size(-1,-1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT,30,0.1));
    }
    if(found1){
      cv::cornerSubPix(gray1,corner1,cv::Size(11,11),cv::Size(-1,-1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT,30,0.1));
    }

    cv::Mat draw0,draw1;
    frame0.copyTo(draw0);
    cv::drawChessboardCorners(draw0,cv::Size(8,6),cv::Mat(corner0),found0);
    frame1.copyTo(draw1);
    cv::drawChessboardCorners(draw1,cv::Size(8,6),cv::Mat(corner1),found1);

    cv::imshow("cam0:"+cam0.getCameraSN()+":draw",draw0);
    cv::imshow("cam1:"+cam1.getCameraSN()+":draw",draw1);
    int key = cv::waitKey(1);
    if(key == 'q')break;
    if(key == 27)break;
    if(key == 's' && found0 && found1){
      corner0_list.push_back(corner0);
      corner1_list.push_back(corner1);
      std::cout<<"corner saved:"<<corner0_list.size()<<std::endl;
    }
  }
  std::vector<cv::Point3f> object;
  for(std::size_t j=0; j < cv::Size(8,6).height; j++){
    for(std::size_t i=0; i < cv::Size(8,6).width; i++){
      object.push_back(cv::Point3f(i*SQ_SIZE,j*SQ_SIZE,0.0f));
    }
  }
  
  std::vector<std::vector<cv::Point3f> > object_list;
  for(std::size_t i=0;i<corner0_list.size();i++){
    object_list.push_back(object);
  }
  cv::Mat R,T,E,F;
  if(cam0.getCameraSN()=="12380091"){
    cv::stereoCalibrate(object_list,corner0_list,corner1_list,K0,D0,K1,D1,frame0.size(),R,T,E,F);
  }else{
    cv::stereoCalibrate(object_list,corner1_list,corner0_list,K0,D0,K1,D1,frame0.size(),R,T,E,F);
  }
  
  fs0.release();
  fs1.release();

  if(cam0.getCameraSN()=="12380091"){
    fs0.open("../data/left_intrinsics.yml",cv::FileStorage::WRITE);
    fs1.open("../data/right_intrinsics.yml",cv::FileStorage::WRITE);
  }else{
    fs1.open("../data/left_intrinsics.yml",cv::FileStorage::WRITE);
    fs0.open("../data/right_intrinsics.yml",cv::FileStorage::WRITE);
  }
  fs0<<"SN"<<cam0.getCameraSN();
  fs0<<"K"<<K0;
  fs0<<"D"<<D0;
  fs1<<"SN"<<cam1.getCameraSN();
  fs1<<"K"<<K1;
  fs1<<"D"<<D1;

  fs0.release();
  fs1.release();
  
  cv::FileStorage fs("../data/stereo_extrinsics.yml",cv::FileStorage::WRITE);
  fs<<"left SN"<<cam0.getCameraSN();
  fs<<"right SN"<<cam1.getCameraSN();
  fs<<"K0"<<K0;
  fs<<"D0"<<D0;
  fs<<"K1"<<K1;
  fs<<"D1"<<D1;
  fs<<"R"<<R;
  fs<<"T"<<T;
  fs<<"E"<<E;
  fs<<"F"<<F;
  fs.release();

  return 0;
}

int mono_calibrate(int argc, char* argv[]){
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
  fs<<"SN"<<cam.getCameraSN();
  fs<<"RMS"<<RMS;
  fs<<"K"<<K;
  fs<<"D"<<D;
  fs.release();
	return 0;
}

int main(int argc, char* argv[]){
  if(STEREO_MATCH){
    stereo_match(argc,argv);
  }else
  if(STEREO_CALIBRATE){
    stereo_calibrate(argc,argv);
  }else
  if(MONO_CALIBRATE){
    mono_calibrate(argc,argv);
  }
  return 0;
}

