#CXX = g++
LD:= g++

CPPFLAGS += -I../common -I/usr/include/pcl-1.7/ -I/usr/include/eigen3/ 
CXX_FLAGS+= -std=c++11 -g -Wall -Wextra -pthread  -fno-strict-aliasing -Wno-unused-parameter 
#EXTRA_FLAGS := -I/home/bongos/bongos_robot/slam/common -I/usr/include/pcl-1.7/
LD_FLAGS+= 

LidarTest: LidarTest.cpp LidarCollector.o macro_defines.o struct_defines.o
	@$(LD) $(CPPFLAGS) -std=c++11 $(LD_FLAGS) $^ -o $@  -lLpSensor -lpthread -ldl -lrt -lm -lz -lboost_system -lpcl_common -lboost_thread
LidarCollector.o:LidarCollector.cpp
	$(CXX) $(CPPFLAGS) $(CXX_FLAGS) -c $< -o $@

macro_defines.o:../common/macro_defines.cpp
	$(CXX) $(CPPFLAGS) $(CXX_FLAGS) -c ../common/$< -o $@

struct_defines.o:../common/struct_defines.cpp
	$(CXX) $(CPPFLAGS) $(CXX_FLAGS) -c ../common/$< -o $@	



clean:
	rm -rf *.o LidarTest





#GNSSOBJ = GnssSingleC.o GnssDoubleC.o  nvsDecoder.o ubloxDecoder.o RTCM.o GnssBasic.o   Satellites.o Correction.o Lambda.o
#$(GNSSOBJ):%.o:gnss/%.cpp
#	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $<