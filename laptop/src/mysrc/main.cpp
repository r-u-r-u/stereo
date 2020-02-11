#include<iostream>
#include<sstream>
#include<vector>
#include<string>

#include"opencv2/opencv.hpp"
#include"opencv2/viz.hpp"
#include"opencv2/sfm.hpp"

int main(int argc,char* argv[]){
  std::string name = "Viz";

  cv::viz::Viz3d myWindow(name);

  myWindow.showWidget("Coordinate Widget",cv::viz::WCoordinateSystem());

  myWindow.spin();
  return 0;
}
