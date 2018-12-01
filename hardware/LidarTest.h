#pragma once

#include "struct_defines.h"

#include "LidarCollector.h"
#include "rslidar/rawdata.h"
#include "rslidar/rsdriver.h"

#include <cmath>
#include <error.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <unistd.h>



enum LidarType {
  kLslidarN301,
  kSickLMS1xx,
  kRobosense16,
  kNoLidar,
  kLidarTypeCount
};

struct SickLmsConfig {
  std::string ip;
  int32_t port;
};

struct DriverConfig {
  // flags
  bool use_lidar;
  bool wait_for_carto;

  // 目前我们支持两个雷达，暂时只支持 2d + 3d
  // 的方式，不支持两个2d雷达或者两个3d雷达的方案（ TODO ）
  LidarType main_lidar_type;
  LidarType the_other_lidar_type;

  // int32_t max_secs_waiting_cmd;
  SickLmsConfig sick_config;
  hardware::rslidar::driver::Config rsdriver_config;
  hardware::rslidar::rawdata::Config rsdriver_rawdata_config;
};
extern DriverConfig default_driver_config;

typedef struct {
  // process
  // 0 ~ 2
  unsigned has_shm : 1;
  unsigned shm_init_finished : 1;
  unsigned lidar_sem_init_finished : 1;
  unsigned lidar_thread_init_finished : 1;
  unsigned lidar_thread_running : 1;
  // 3 ~ 4
  unsigned carto_sem_init_finished : 1;
  unsigned shm_sem_init_finished : 1;

  // 5 ~ 9
  unsigned motor_thread_init_finished : 1;
  unsigned motor_thread_running : 1;

  unsigned soft_interrupt_thread_init_finished : 1;
  unsigned soft_interrupt_thread_running : 1;
  unsigned imu_sem_init_finished : 1;

  // 10 ~ 14
  // hardware ready
  unsigned imu_ready : 1;
  unsigned motor_ready : 1;
  unsigned main_lidar_ready : 1;
  unsigned the_other_lidar_ready : 1;
  unsigned gps_ready : 1;

  // 15 ~ 20
  unsigned save_bag_thread_init_finished : 1;
  unsigned save_bag_thread_running : 1;
  unsigned save_bag_sem_init_finished : 1;
  unsigned has_loaded_bag : 1;
  unsigned imu_thread_init_finished : 1;
  unsigned imu_thread_running : 1;

  // 21 ~ 22
  unsigned load_bag_thread_init_finished : 1;
  unsigned load_bag_thread_running : 1;

  unsigned gps_sem_init_finished : 1;

  unsigned reserved : 8;

} DriverStatus;
extern DriverStatus driver_status;

// functions


void kill_lidar_thread();
void create_and_init_shm();
void destroy_shm();
void segment_fault_handler(int32_t sig);
void ctrl_c_handler(int32_t sig);

DriverConfig load_config(int argv, char **argc);

void process_exit(DriverStatus *status);

int32_t init_lidar(void);


void *lidar_data_update(void *);
