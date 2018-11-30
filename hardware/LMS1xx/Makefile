OBJS = LMS1xx_driver_test.o LMS1xx.o struct_defines.o
CXX = g++ -s -O2 -DNDEBUG -Wno-unused-result -std=c++11
TARGET = driver_lsm1xx
CFLAGS =  -I/usr/include/pcl -I/usr/include/eigen3 -I../../common



all: $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET) $(CFLAGS) -lpcl_common
	
LMS1xx.o: LMS1xx.cpp
	$(CXX) -c LMS1xx.cpp -o LMS1xx.o $(CFLAGS)
LMS1xx_driver_test.o: LMS1xx_driver_test.cc
	$(CXX) -c LMS1xx_driver_test.cc -o LMS1xx_driver_test.o $(CFLAGS)
struct_defines.o: ../../common/struct_defines.cpp
	$(CXX) -c ../../common/struct_defines.cpp -o struct_defines.o $(CFLAGS)
	
clean:
	rm -f *.o
	rm -f $(TARGET)
	
