
ifeq ($(sycl),true)
  SYCL_FLAG = -DMATRIX_USE_SYCL
else
  SYCL_FLAG = 
endif

SRC_DIR := .
OBJ_DIR := obj
CXXFLAGS += -O3 -DNDEBUG -std=c++11 $(SYCL_FLAG)
EXTRA_FLAGS = -fopenmp
ARFLAGS = rv

#### for SYCL (openCL) ####
COMPUTECPP_PREFIX ?= /home/bongos/ComputeCpp-CE-0.7.0
COMPUTECPP ?= $(COMPUTECPP_PREFIX)/bin/compute++
COMPUTECPP_INFO ?= $(COMPUTECPP_PREFIX)/bin/computecpp_info
COMPUTECPP_INCLUDES ?= $(COMPUTECPP_PREFIX)/include
COMPUTECPP_LIBS ?= $(COMPUTECPP_PREFIX)/lib
# In addition your normal flags, compilation requires C++11 standard,
# the SYCL headers, and the ComputeCpp library.
SYCL_CXXFLAGS += -std=c++11 -DNDEBUG -I$(COMPUTECPP_INCLUDES) $(SYCL_FLAG)
LDFLAGS += -L$(COMPUTECPP_LIBS) -lComputeCpp -Wl,--rpath $(COMPUTECPP_LIBS)
COMPUTECPP_FLAGS += -no-serial-memop $(SYCL_CXXFLAGS) \
    $(shell $(COMPUTECPP_INFO) --dump-device-compiler-flags)

LIB_OBJS = 	$(OBJ_DIR)/icp.o \
		$(OBJ_DIR)/icpPointToPlane.o \
		$(OBJ_DIR)/icpPointToPoint.o \
		$(OBJ_DIR)/kdtree.o   
LIB_TARGET = 	libicp.a
NORMAL_OBJS = 	$(LIB_OBJS) \
		$(OBJ_DIR)/demo.o
SYCL_OBJS = 	$(OBJ_DIR)/matrix.o

all: $(NORMAL_OBJS) $(SYCL_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o icp $(EXTRA_FLAGS) $(LDFLAGS)

lib: $(LIB_OBJS) $(SYCL_OBJS)
	$(AR) $(ARFLAGS) $(LIB_TARGET) $^
	
ifeq ($(sycl),true)

$(NORMAL_OBJS):$(OBJ_DIR)/%.o:$(SRC_DIR)/%.cpp
	${CXX} $(CXXFLAGS) -I$(COMPUTECPP_INCLUDES) -c $< -o $@ $(EXTRA_FLAGS)
	
$(SYCL_OBJS):$(OBJ_DIR)/%.o:$(SRC_DIR)/%.cpp
	$(COMPUTECPP) $(COMPUTECPP_FLAGS) -c $< -o $@.sycl
	$(CXX) $(CXXFLAGS) -I$(COMPUTECPP_INCLUDES) -include $@.sycl -c $< -o $@ $(EXTRA_FLAGS)

else
$(OBJ_DIR)/%.o : $(SRC_DIR)/%.cpp
	${CXX} $(CXXFLAGS) -c $< -o $@ $(EXTRA_FLAGS)
endif
	
clean:
	rm -rf $(OBJ_DIR)/*
	rm -rf *.a
