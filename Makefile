EXTRA_FLAGS := -Ithird_parties -Islam -Icommon
CXXFLAGS += -std=c++11 -Wall -Wno-unused-result
OBJDIR = generated_files
DOCDIR = doc
COMMON_DIR := common
TOOLCHAIN_DIR = /opt/arm-gcc-5.4.1/arm-linux-gnueabihf

DRIVER_TARGET = LidarTest

# if there is no obj dir, create it
ifeq ($(OBJDIR), $(wildcard $(OBJDIR)))
  # do nothing
else
  $(shell mkdir -p $(OBJDIR))
endif

ECHO_GREEN = "\033[33m"
ECHO_BLUE = "\033[34m"
ECHO_NONE = "\033[0m"
ECHO_RED = "\033[31m"
ECHO_BOLD = "\033[1m"

# find pcl
# only pcl1.7 / pcl-1.8 supported
PCL_INCLUDE := /usr/include/pcl
ifeq ($(PCL_INCLUDE), $(wildcard $(PCL_INCLUDE)))
  # do nothing
else
  ifeq (/usr/include/pcl-1.7, $(wildcard /usr/include/pcl-1.7))
    PCL_INCLUDE := /usr/include/pcl-1.7
  else
    ifeq (/usr/include/pcl-1.8, $(wildcard /usr/include/pcl-1.8))
      PCL_INCLUDE := /usr/include/pcl-1.8
    else 
      $(error Did not found pcl include dir )
    endif
  endif
endif

.PHONY:all
# all: $(DRIVER_TARGET) $(CARTO_TARGET) $(STARTER_TARGET)
all:
	#
	@ # using "$(MAKE)" instead of using "make" directly so that "-j4" is available
	@ # for faster compiling
	$(MAKE) $(DRIVER_TARGET)
	# 
	#
	$(MAKE) $(CARTO_TARGET)
	#
	#
	$(MAKE) $(STARTER_TARGET)
	#
	
# 自动生成头文件依赖关系的函数定义
generate_d_file = @set -e; rm -f $(1); \
  echo $(ECHO_BLUE)"Building dependency: $(1)"$(ECHO_NONE); \
  $(GCC) $(CXXFLAGS) $(2) -MM -MT $(patsubst %.d,%.o,$(1)) $(3) > $(1).tmp; \
  sed 's,\($*\)\.o[ :]*,\1.o $(1) : ,g' < $(1).tmp > $(1); \
  rm -f $(1).tmp; sync;

####### drivers ########
### 自动获取 driver 所需的所有源文件
DRIVER_DIR := hardware
INCDIR_N301 := $(DRIVER_DIR)/lslidar_n301

DRIVER_SRCS := $(shell find $(DRIVER_DIR) -path "$(INCDIR_N301)" -prune -o -type f)
DRIVER_SRCS += $(wildcard $(COMMON_DIR)/*.cpp )
DRIVER_SRCS := $(filter %.c %.cpp %.cc,$(DRIVER_SRCS))
DRIVER_SRCS := $(filter-out %_test.cc,$(DRIVER_SRCS))
DRIVER_OBJS := $(foreach src_file, $(DRIVER_SRCS), $(OBJDIR)/$(patsubst %.cpp,%.o,$(notdir $(src_file))))
DRIVER_OBJS := $(foreach src_file, $(DRIVER_OBJS), $(OBJDIR)/$(patsubst %.cc,%.o,$(notdir $(src_file))))
DRIVER_OBJS := $(foreach src_file, $(DRIVER_OBJS), $(OBJDIR)/$(patsubst %.c,%.o,$(notdir $(src_file))))
DRIVER_INCS := $(PCL_INCLUDE) /usr/include/eigen3 common third_parties
DRIVER_INCS := $(foreach d, $(DRIVER_INCS), -I$d)
DRIVER_LIBS := -lLpSensor -lpthread -ldl -lrt -lm -lz -lboost_system -lpcl_common -lboost_thread
ifeq ($(arch),arm)
  DRIVER_LIBS += $(CROSS_COMPILE_LIB_DIR) 
endif

$(DRIVER_TARGET): $(DRIVER_OBJS)
	@echo $(ECHO_GREEN)"\n  Targeting: "$@"\n"$(ECHO_NONE)
	@ $(GCC) $^ -o $(DRIVER_TARGET) $(DRIVER_LIBS) 

$(OBJDIR)/%.d: $(DRIVER_DIR)/%.cpp
	$(call generate_d_file, $@, $(DRIVER_INCS), $<)
$(OBJDIR)/%.o: $(DRIVER_DIR)/%.cpp
	@echo $(ECHO_GREEN)"Building: "$@$(ECHO_NONE)
	@ $(GCC) -c $(CXXFLAGS) $(DRIVER_INCS) $< -o $@

## LMS1xx
INCDIR_LMS := $(DRIVER_DIR)/LMS1xx
$(OBJDIR)/%.d: $(INCDIR_LMS)/%.cpp
	$(call generate_d_file, $@, $(DRIVER_INCS), $<)
$(OBJDIR)/%.o: $(INCDIR_LMS)/%.cpp
	@echo $(ECHO_GREEN)"Building: "$@$(ECHO_NONE)
	@ $(GCC) -c $(CXXFLAGS) $(DRIVER_INCS) $< -o $@

## RoboSense RS-16
RS_LIDAR_SRC := $(DRIVER_DIR)/rslidar
$(OBJDIR)/%.d: $(RS_LIDAR_SRC)/%.cc
	$(call generate_d_file, $@, $(DRIVER_INCS), $<)
$(OBJDIR)/%.o: $(RS_LIDAR_SRC)/%.cc
	@echo $(ECHO_GREEN)"Building: "$@$(ECHO_NONE)
	@ $(GCC) -c $(CXXFLAGS) $(DRIVER_INCS) $< -o $@