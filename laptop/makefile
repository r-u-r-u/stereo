CXX = g++
TARGET = CV_learn
SRCS = $(wildcard src/*.cpp)
CXXFLAGS = -Wall -O2 -pipe -std=c++11 $(USR_DEFINE)
LIBS = `pkg-config --libs opencv`
INCLUDES = -I./include -I/usr/local/include/opencv4 `pkg-config --cflags eigen3` 
DEFINES = -DCERES_FOUND=1
OBJS := $(SRCS:.cpp=.o)

$(TARGET):$(SRCS)
	$(CXX) -o $@ $(SRCS) $(CXXFLAGS) $(LIBS) $(INCLUDES) $(DEFINES)
 
clean:
	rm -f $(TARGET) $(OBJS)
