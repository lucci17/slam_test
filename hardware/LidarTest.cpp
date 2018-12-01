#include "LidarTest.h"
#include "macro_defines.h"
#include "process.h"


using namespace Bgs;
using namespace hardware;
volatile bool b_process_running = true;
static const int scan_size = SCAN_SIZE;
extern int errno;

std::shared_ptr<LidarCollector> main_lidar_collector = nullptr;

LaserScanPtr laser_scan_msg_ptr = nullptr;
PointCloud2Ptr point_cloud_ptr = nullptr;
pthread_t lidar_thread = 0;
int32_t lock_fd = 0;

SickLmsConfig lms_config ={.ip = "10.0.0.20",.port = 2111}; 
DriverConfig driver_config = {.use_lidar = true,
                              .wait_for_carto = true,
                              .main_lidar_type = kSickLMS1xx,
                              .the_other_lidar_type = kNoLidar, 
                              .sick_config = lms_config};
DriverStatus driver_status;
int32_t init_lidar(void)
{
  // LIDAR
  // 雷达是整个驱动的基础，雷达若未初始化成功则整个驱动直接退出，不需要再运行
  // 雷达本身的数据更新频率作为整个驱动的心跳
  // 根据配置信息来创建响应的雷达对象
  driver_status.main_lidar_ready = FALSE;
  driver_status.the_other_lidar_ready = FALSE;

  if (!driver_config.use_lidar)
    return BGS_OK;

  std::function<void(void)> error_func = []() {
    PRINT_DEBUG("lidar error!!!");
    // g_ptr_shm->error.lidar_error = TRUE;
  };

  const std::string lidar_frame_id = "horizontal_laser_link";
  switch (driver_config.main_lidar_type) {
  case kLslidarN301:
    break;

  case kSickLMS1xx:

    laser_scan_msg_ptr = std::make_shared<LaserScanMsg>();
    laser_scan_msg_ptr->header.frame_id = lidar_frame_id;
    main_lidar_collector = std::make_shared<LMS1Collector>();

    if (!main_lidar_collector) 
    {
      PRINT_ERROR("Lidar construct failed!");
      return BGS_ERROR;
    }
    
    if (main_lidar_collector->Init(
          (char *)(driver_config.sick_config.ip.c_str()),
          driver_config.sick_config.port)
        == BGS_OK)
    {
      driver_status.main_lidar_ready = TRUE;
      // PRINT_DEBUG("main lader ready");
      return BGS_OK;
    }
    else {
      PRINT_ERROR("failed to init lidar!");
      main_lidar_collector->DeInit();
      return BGS_ERROR;
    }
    break;

  case kRobosense16:
  case kNoLidar:
  default:

    PRINT_ERROR_FMT("Unkown lidar type: %d", driver_config.main_lidar_type);
    return BGS_ERROR;
  }

  if (main_lidar_collector)
    main_lidar_collector->on_error_ = error_func;

  PRINT_INFO("lidar init succeed!");
  return BGS_OK;
}

void *lidar_data_update(void *)
{
  if (!main_lidar_collector || !driver_status.main_lidar_ready) {
    pthread_exit(NULL);
  }

  if (driver_config.main_lidar_type == kSickLMS1xx) {
    PRINT_INFO("It is a lms1xx sensor");
    driver_status.lidar_thread_running = TRUE;
    int32_t error_count = 0;
    const int32_t max_error_count = 2000;
    const int32_t sleep_time_us = 10;
    main_lidar_collector->GetScanCfg(laser_scan_msg_ptr.get());
    while (b_process_running) {
      // PRINT_DEBUG(b_process_running);
      // if (driver_config.wait_for_carto && g_ptr_shm->n_carto_pid == 0) {
      //   usleep(sleep_time_us);
      //   continue;
      // }
      if (main_lidar_collector->GetScanMsg(laser_scan_msg_ptr.get()) == BGS_ERROR)
      {
        error_count++;
        if (error_count >= max_error_count)
        {
          b_process_running = false;
          break;
        }
        else {
          usleep(sleep_time_us);
          continue;
        }
      }
    //    usleep( sleep_time_us );
    }
  }

  PRINT_INFO("lidar thread exit!");
  driver_status.lidar_thread_running = FALSE;

  pthread_exit(NULL);
}

int main(int argv, char **argc)
{
  if (!common::is_single_process("driver.lck", lock_fd)) {
    PRINT_ERROR("driver is already running.");
    return -1;
  }

  // step0 create and init shared memory

  if (init_lidar() == BGS_ERROR) {
      // process_exit(&driver_status);
      return -1;
  }

  //init driver thread
  driver_status.lidar_thread_init_finished = FALSE;
  driver_status.lidar_thread_running = FALSE;
  if (driver_config.use_lidar) 
  {
    if (pthread_create(&lidar_thread, NULL, lidar_data_update, NULL)) {
      PRINT_ERROR("Create lidar thread failed!");
      // process_exit(&driver_status);
      return -1;
    }
    driver_status.lidar_thread_init_finished = TRUE;
  }

  while(b_process_running){}
  // step3 finish process
  PRINT_INFO("Main Thread quit!");
  return 0;
}

