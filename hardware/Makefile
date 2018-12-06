LD:= g++
ARFLAGS = r
OBJDIR := generated_files
COMMON_DIR := common
CPPFLAGS += -I../common -I/usr/include/pcl-1.7/ -I/usr/include/eigen3/ 
CXX_FLAGS+= -std=c++11 -g -Wall -Wextra -pthread  -fno-strict-aliasing -Wno-unused-parameter 
#EXTRA_FLAGS := -I/home/bongos/bongos_robot/slam/common -I/usr/include/pcl-1.7/
LD_FLAGS+= 

# if there is no obj dir, create it
ifeq ($(OBJDIR), $(wildcard $(OBJDIR)))
  # do nothing
else
  $(shell mkdir -p $(OBJDIR))
endif

LidarTest: LidarTest.cpp LMS1xx.a macro_defines.o struct_defines.o input.o rawdata.o convert.o rsdriver.o LidarCollector.o
	@$(LD) $(CPPFLAGS) -std=c++11 $(LD_FLAGS) $^ -o $@  -lLpSensor -lpthread -ldl -lrt -lm -lz -lboost_system -lpcl_common -lboost_thread


macro_defines.o:../common/macro_defines.cpp
	$(CXX) $(CPPFLAGS) $(CXX_FLAGS) -c ../common/$< -o $@

struct_defines.o:../common/struct_defines.cpp
	$(CXX) $(CPPFLAGS) $(CXX_FLAGS) -c ../common/$< -o $@	


LMSOBJ = $(OBJDIR)/LMS1xx.o
$(LMSOBJ):$(OBJDIR)/%.o:./LMS1xx/%.cpp
	$(CXX) $(CPPFLAGS) $(CXX_FLAGS) -c $< -o $@

LMS1xx.a:$(LMSOBJ)
	$(AR) $(ARFLAGS) $@ $^

# OSOBJ = $(OBJDIR)/input.o $(OBJDIR)/rawdata.o $(OBJDIR)/convert.o $(OBJDIR)/rsdriver.o   
# RS_LIDAR_SRC := ./rslidar
# $(OSOBJ):$(OBJDIR)/%.o: $(RS_LIDAR_SRC)/%.cc
# 	$(CXX) $(CPPFLAGS) $(CXX_FLAGS) -c $< -o $@

# RSlib.a:$(OSOBJ)
# 	$(AR) $(ARFLAGS) $@ $^

input.o:./rslidar/input.cc
	$(CXX) $(CPPFLAGS) $(CXX_FLAGS) -c $< -o $@

rawdata.o:./rslidar/rawdata.cc
	$(CXX) $(CPPFLAGS) $(CXX_FLAGS) -c $< -o $@

convert.o:./rslidar/convert.cc 
	$(CXX) $(CPPFLAGS) $(CXX_FLAGS) -c $< -o $@

rsdriver.o:./rslidar/rsdriver.cc input.o
	$(CXX) $(CPPFLAGS) $(CXX_FLAGS) -c $< -o $@

LidarCollector.o:./LidarCollector.cpp 
	$(CXX) $(CPPFLAGS) $(CXX_FLAGS) -c $< -o $@

clean:
	rm -rf *.o *.a LidarTest





#GNSSOBJ = GnssSingleC.o GnssDoubleC.o  nvsDecoder.o ubloxDecoder.o RTCM.o GnssBasic.o   Satellites.o Correction.o Lambda.o
#$(GNSSOBJ):%.o:gnss/%.cpp
#	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $<