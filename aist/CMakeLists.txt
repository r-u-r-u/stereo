cmake_minimum_required(VERSION 2.8)

add_definitions(-Wall)
add_definitions(-std=c++11)

add_compile_options(-DMONO_CALIBRATE=0)
add_compile_options(-DSTEREO_CALIBRATE=0)
add_compile_options(-DSTEREO_MATCH=1)

find_package(OpenCV REQUIRED)

add_executable(main 
	src/main.cpp
	src/FlyCap2CV.cpp
)

include_directories(
  include
	${OpenCV_INCLUDE_DIRS}
  /usr/include
)
target_link_libraries(main
	${OpenCV_LIBRARIES}
  flycapture
)
