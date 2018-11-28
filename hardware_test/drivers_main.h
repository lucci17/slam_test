#pragma once

#include "struct_defines.h"

#include "IMUCollector.h"
#include "LidarCollector.h"
#include "Motor.h"
#include "data_bag.h"

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
  bool is_valid;
  bool use_imu;
  bool use_motor;
  bool use_gps;
  bool use_lidar;
  bool use_soft_interrput;
  bool save_data_bag;
  bool load_data_bag;
  bool wait_for_carto;

  // 目前我们支持两个雷达，暂时只支持 2d + 3d
  // 的方式，不支持两个2d雷达或者两个3d雷达的方案（ TODO ）
  LidarType main_lidar_type;
  LidarType the_other_lidar_type;
  ImuType imu_type;
  MotorType motor_type;

  std::string imu_dev_path;
  std::string gps_dev_path;

  std::string motor_dev_path1;
  std::string motor_dev_path2;

  std::string data_bag_filaname; // 保存时使用的文件名
  std::string load_bag_filename; // 载入的bag文件名

  int32_t max_secs_waiting_cmd;
  int32_t imu_update_freq;
  int32_t motor_update_freq;
  int32_t load_bag_percentage; // 0 - 100

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

void kill_soft_interrupt_thread();
void kill_lidar_thread();
void kill_motor_thread();
void kill_imu_thread();
void kill_save_bag_thread();
void kill_load_bag_thread();
void create_and_init_shm();
void destroy_shm();
void segment_fault_handler(int32_t sig);
void ctrl_c_handler(int32_t sig);
bool imu_msg_valid(const Bgs::ImuMsgPtr &imu_msg);

DriverConfig load_config(int argv, char **argc);

void process_exit(DriverStatus *status);

int32_t init_signals(void);
int32_t init_motor(void);
int32_t init_gps(void);
int32_t init_imu(void);
int32_t init_lidar(void);
int32_t init_sems(void);

void lpms_callback(ImuData data, const char *id);
void *imu_data_update(void *);
void *lidar_data_update(void *);
void *soft_interrupt(void *);
void *data_bag_update(void *);
void *motor_data_update(void *);
void *load_bag_data(void *);

// DRIVER_MAIN_H
class LidarThreadManage
{
public:
  LidarThreadManage(){};
  ~LidarThreadManage(){};
  int32_t Init();
  int32_t UpdateData();
  void DeInit();
private:
  char *main_lidar;
  char *the_other_lidar;
};