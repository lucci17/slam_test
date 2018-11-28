#include "drivers_main.h"
#include "GPSCollector.h"
#include "macro_defines.h"
#include "process.h"
#include "pugixml/pugiconfig.hpp"
#include "pugixml/pugixml.hpp"
#include <boost/make_shared.hpp>
#include <fstream>
#include <mutex>
#include <vector>
#include <zlib.h>

using namespace Bgs;
using namespace hardware;
using namespace hardware::databag;
using namespace pugi;
// using namespace std;

volatile bool b_process_running = true;
static const int scan_size = SCAN_SIZE;
extern int errno;

std::shared_ptr<LidarCollector> main_lidar_collector = nullptr;
std::shared_ptr<LidarCollector> the_other_lidar_collector = nullptr;
std::shared_ptr<IMUCollector> imu_collector = nullptr;
std::shared_ptr<GPSCollector> gps_collector = nullptr;
std::shared_ptr<Motor> motor = nullptr;

LaserScanPtr laser_scan_msg_ptr = nullptr;
ImuMsgPtr imu_msg_ptr = nullptr;
MotorMsgPtr motor_msg_ptr = nullptr;
PointCloud2Ptr point_cloud_ptr = nullptr;

GlobalShm *g_ptr_shm = NULL;
static int32_t local_carto_sem_id = 0;
static int32_t local_imu_sem_id = 0;
static int32_t local_gps_sem_id = 0;
static int32_t local_lidar_sem_id = 0;
static int32_t local_save_bag_sem_id = 0;
static bool b_shm_get_succeed = false;

std::vector<ImuMsgInShm> saved_imu_for_bag;
pthread_mutex_t mutex_for_imu_vec = PTHREAD_MUTEX_INITIALIZER;

// save data bag
Byte *bag_data = NULL;
int32_t bag_file_size = 0;
int32_t bag_data_offset = 0;

sem_t lidar_sem;
int32_t lock_fd = 0;

pthread_t lidar_thread = 0;
// threads

DriverConfig default_driver_config = {.is_valid = true,
                                      .use_imu = false,
                                      .use_motor = false,
                                      .use_gps = false,
                                      .use_lidar = true,
                                      .use_soft_interrput = false,
                                      .save_data_bag = false,
                                      .load_data_bag = false,
                                      .wait_for_carto = true,
                                      .main_lidar_type = kSickLMS1xx,
                                      .the_other_lidar_type = kNoLidar,
                                      .imu_type = kLpms,
                                      .motor_type = kDoubleGeek,
                                      .imu_dev_path = "/dev/ttyUSB0",
                                      .gps_dev_path = "/dev/ttyUSB0",
                                      .motor_dev_path1 = "/dev/ttyS4",
                                      .motor_dev_path2 = "/dev/ttyS5",
                                      .data_bag_filaname = "sensor_data.bag",
                                      .load_bag_filename = "sensor_data.bag",
                                      .max_secs_waiting_cmd = 20,
                                      .imu_update_freq = 50,
                                      .motor_update_freq = 10,
                                      .load_bag_percentage = 100 };
int driver_count = 0;

DriverConfig driver_config;
DriverStatus driver_status;


int main(int argv, char **argc)
{
  // std::cout<<"[debug]start anyway!\n";
  if (!common::is_single_process("driver.lck", lock_fd)) {
    PRINT_ERROR("driver is already running.");
    return -1;
  }

  // step0 create and init shared memory
  create_and_init_shm();
  if (!b_shm_get_succeed) {
    PRINT_ERROR(" shm init error!");
    return -1;
  }

  bool b_error_before_init_drivers = false;
  driver_config = load_config(argv, argc);
  if (driver_config.is_valid == false) {
    PRINT_ERROR("Config failed!");
    b_error_before_init_drivers = true;
  }

  if (init_signals() != 0) {
    PRINT_ERROR("Signals error!");
    b_error_before_init_drivers = true;
  }

  if (init_sems() == BGS_ERROR) {
    PRINT_ERROR("sems init error!");
    b_error_before_init_drivers = true;
  }

  if (b_error_before_init_drivers) {
    destroy_shm();
    exit(-1);
  }

  driver_status.lidar_thread_init_finished = FALSE;
  driver_status.lidar_thread_running = FALSE;
  if (driver_config.use_lidar) 
  {
    if (pthread_create(&lidar_thread, NULL, lidar_data_update, NULL)) {
      PRINT_ERROR("Create lidar thread failed!");
      process_exit(&driver_status);
      return -1;
    }
    driver_status.lidar_thread_init_finished = TRUE;
  }
  // 线程间同步，且初始化为 0
  // 这样初始化的信号量在进程结束时会自动释放
  // driver_status.lidar_sem_init_finished = FALSE;
  // if (sem_init(&lidar_sem, 0, 0) != 0) {
  //   PRINT_ERROR("lidar sem init failed!");
  //   process_exit(&driver_status);
  //   return -1;
  // }
  // driver_status.lidar_sem_init_finished = TRUE;

  g_ptr_shm->driver_status.all_ready = TRUE;

  // step2 get sensor data and write them to the shared memory
  int32_t sample_count = 200;
  int32_t i = 0;
  struct timespec ts;
  int32_t sem_error_count = 0;
  while (b_process_running) {
  //   // step1 等待信号量（来自 lidar thread）
  //   // 超时则判断 lidar thread 是否还在运行，如果没有运行则直接退出
  //   clock_gettime(CLOCK_REALTIME, &ts);
  //   ts.tv_sec += 1; // 超时1秒
  //   if (sem_timedwait(&lidar_sem, &ts) != 0)
  //     continue;

  //   if (driver_status.save_bag_thread_running) {
  //     static int32_t save_bag_sem_error_count = 0;
  //     if (semaphore_v(local_save_bag_sem_id) < 0) {
  //       save_bag_sem_error_count++;
  //       if (save_bag_sem_error_count >= 20) {
  //         PRINT_ERROR("sem error for several times");
  //       }
  //     }
  //   }

  //   // 信号量多次出错之后直接停止 driver 的运行
  //   // 暂定为 20 次
  //   if (driver_config.wait_for_carto && semaphore_v(local_carto_sem_id) < 0) {
  //     sem_error_count++;
  //     if (sem_error_count >= 20) {
  //       PRINT_ERROR("sem error for several times");
  //       break;
  //     }
  //   }

  //   if (driver_config.wait_for_carto && driver_config.use_gps
  //       && semaphore_v(local_gps_sem_id) < 0) {
  //     sem_error_count++;
  //     if (sem_error_count >= 20) {
  //       PRINT_ERROR("sem error for several times");
  //       break;
  //     }
  //   }

  //   i++;
  //   if (!driver_config.load_data_bag && i >= sample_count) {
  //     PRINT_DEBUG_FMT("lidar recieved for %d times!", sample_count);
  //     i = 0;
  //   }
    PRINT_DEBUG("lidar thread running");
  }

  // step3 finish process
  process_exit(&driver_status);
  PRINT_INFO("Main Thread quit!");
  return 0;
}

void process_exit(DriverStatus *status)
{
  b_process_running = false;

  PRINT_INFO("all threads have been killed!");

  PRINT_INFO("destroy imu mutex");
  pthread_mutex_destroy(&mutex_for_imu_vec);

  if (status->has_loaded_bag) {
    close_bag_file(bag_data, bag_file_size);
    status->has_loaded_bag = FALSE;
  }

  // deinit all the available hardware
  if (gps_collector && status->gps_ready) {
    PRINT_DEBUG("deinit gps");
    gps_collector->deInit();
  }
  if (imu_collector && status->imu_ready) {
    PRINT_DEBUG("deinit imu");
    imu_collector->deInit();
  }
  if (main_lidar_collector && status->main_lidar_ready) {
    PRINT_DEBUG("deinit the main lidar");
    main_lidar_collector->DeInit();
  }
  if (the_other_lidar_collector && status->the_other_lidar_ready) {
    PRINT_DEBUG("deinit the other lidar");
    the_other_lidar_collector->DeInit();
  }
  if (motor && status->motor_ready) {
    PRINT_DEBUG("deinit motor");
    motor->deInit();
  }

  if (status->shm_init_finished)
    destroy_shm();
}

void kill_lidar_thread()
{
  if (driver_status.lidar_thread_init_finished && lidar_thread != 0)
    pthread_join(lidar_thread, NULL);
  driver_status.lidar_thread_init_finished = FALSE;
}

/********************** 以下这几个函数完全是为读写数据包服务的
 * *************************/
// 默认data里面有数据，shm也非空，不再作多余的判断
// 返回的是所有数据里面最靠后的时间戳
typedef std::pair<int32_t /* available_sensors*/, GlobalShm /* sensor data*/>
  AvailableSensors;
int32_t sensor_data_to_globalshm(SensorData *data, GlobalShm *shm)
{
  BgsTime time;

  int available_sensors = data->available_sensors_;
  *shm = *(GlobalShm *)(data->uncompressed_data_);
  if (available_sensors & BAG_LASER_AVAILABLE)
    time = shm->scan_msg.header.stamp;
  if ((available_sensors & BAG_IMU_AVAILABLE)
      && (shm->imu_msg.header.stamp > time))
    time = shm->imu_msg.header.stamp;
  if ((available_sensors & BAG_ODOM_AVAILABLE)
      && (shm->odom_msg.header.stamp > time))
    time = shm->odom_msg.header.stamp;
  if ((available_sensors & BAG_GPS_AVAILABLE)
      && (shm->gps_msg.header.stamp > time))
    time = shm->gps_msg.header.stamp;

  // 马达的时间戳比较特别，它是单独的线程，时间戳跟 lidar
  // 的时间戳并没有直接的关系
  // 	if( (available_sensors & BAG_MTR_AVAILABLE) && (
  // shm->motor_msg.header.stamp > time ) )
  // 		time = shm->motor_msg.header.stamp;

  shm->stamp = time;
  return available_sensors;
}

BgsTime get_time_stamp_of_global(AvailableSensors &data)
{
  BgsTime max_time;
  if ((data.first & BAG_LASER_AVAILABLE)
      && max_time < data.second.scan_msg.header.stamp)
    max_time = data.second.scan_msg.header.stamp;
  if ((data.first & BAG_IMU_AVAILABLE)
      && max_time < data.second.imu_msg.header.stamp)
    max_time = data.second.imu_msg.header.stamp;
  if (/* ( data.first & BAG_POINT_CLOUD_AVAILABLE ) &&*/ max_time
      < data.second.point_cloud2_msg.header.stamp)
    max_time = data.second.point_cloud2_msg.header.stamp;

  return max_time;
}

// 将数据包里的数据的时间戳对齐到 real time
void align_time_stamp(AvailableSensors &data, BgsTime delta_time)
{
  if (data.first & BAG_LASER_AVAILABLE)
    data.second.scan_msg.header.stamp += delta_time;
  if (data.first & BAG_POINT_CLOUD_AVAILABLE)
    data.second.point_cloud2_msg.header.stamp += delta_time;
  if (data.first & BAG_IMU_AVAILABLE)
    data.second.imu_msg.header.stamp += delta_time;
  if (data.first & BAG_GPS_AVAILABLE)
    data.second.gps_msg.header.stamp += delta_time;
  if (data.first & BAG_ODOM_AVAILABLE)
    data.second.odom_msg.header.stamp += delta_time;
  if (data.first & BAG_MTR_AVAILABLE)
    data.second.motor_msg.header.stamp += delta_time;
}
// 一帧数据写入到共享内存里
void write_data_to_shm(AvailableSensors &data)
{
  LOCK_SHM;

  if (data.first & BAG_LASER_AVAILABLE)
    g_ptr_shm->scan_msg = data.second.scan_msg;
  // 	if( data.first & BAG_IMU_AVAILABLE )
  // 		g_ptr_shm->imu_msg = data.second.imu_msg;
  if (data.first & BAG_GPS_AVAILABLE)
    g_ptr_shm->gps_msg = data.second.gps_msg;
  if (data.first & BAG_ODOM_AVAILABLE)
    g_ptr_shm->odom_msg = data.second.odom_msg;
  if (data.first & BAG_MTR_AVAILABLE)
    g_ptr_shm->motor_msg = data.second.motor_msg;
  if (data.first & BAG_POINT_CLOUD_AVAILABLE)
    g_ptr_shm->point_cloud2_msg = data.second.point_cloud2_msg;

  UNLOCK_SHM;
}
/***********************************************************************************/


#define IS_NEW_CMD 1
#define NOT_NEW_CMD 0



void create_and_init_shm()
{
  driver_status.shm_init_finished = FALSE;
  b_shm_get_succeed = false;
  key_t key = BGS_SHM_KEY;
  const size_t size_of_shm = sizeof(GlobalShm);
#ifndef __arm__
  PRINT_INFO_FMT("Size of shared memory = %ld", size_of_shm);
#else
  PRINT_INFO_FMT("Size of shared memory = %u", size_of_shm);
#endif

  // 首先确保这块内存已经存在
  // 如果这种打开方式报错，则证明该共享内存已经存在，再进行下一步的操作
  // 如果创建成功则证明该共享内存事先不存在，则返回错误
  int shm_id = shmget(key, size_of_shm, 0666 | IPC_CREAT | IPC_EXCL);
  if (shm_id >= 0) {
    shmctl(shm_id, IPC_RMID, NULL);
    PRINT_DEBUG("shared memory is not existed! Please RUN \"starter\" first! ");
    return;
  }

  // 最后的flag确保在已经存在这一块共享内存的情况下不再创建
  shm_id = shmget(key, size_of_shm, 0666 | IPC_CREAT);
  if (shm_id < 0) {
    switch (errno) {
    case EINVAL:
      PRINT_ERROR("Shm get failed! : size");
      break;

    case ENOMEM:
      PRINT_ERROR("Shm get failed! : Running out of memory");
      break;

    default:
      PRINT_ERROR("Shm get failed! ( default )");
      break;
    }
    perror("[CARTO ERROR] Shm get error ");
    return;
  }

  g_ptr_shm = (GlobalShm *)shmat(shm_id, NULL, 0);
  driver_status.shm_init_finished = TRUE;
  driver_status.carto_sem_init_finished =
    g_ptr_shm->carto_sem_id > 0 ? TRUE : FALSE;
  local_carto_sem_id = g_ptr_shm->carto_sem_id;
  driver_status.shm_sem_init_finished =
    g_ptr_shm->shm_sem_id > 0 ? TRUE : FALSE;
  b_shm_get_succeed = true;
  return;
}

void destroy_shm()
{
  if (!b_shm_get_succeed || !g_ptr_shm)
    return;

  // 共享内存的开辟和释放不由本进程管理，所以只做 detach 操作，不去释放内存
  if (g_ptr_shm != NULL) {
    g_ptr_shm->n_carto_pid = 0;
    if (shmdt(g_ptr_shm) == -1) {
      perror("[Liuyc] Detach error :");
    } else {
      PRINT_INFO("detached shared memory!");
    }
    g_ptr_shm = NULL;
  }

  common::reset_lock_file(lock_fd);
}

// 程序异常退出（段错误）的处理
void segment_fault_handler(int32_t sig)
{
  PRINT_ERROR("Segmentation fault!");
  // 如果共享内存已经开启，则释放掉共享内存
  destroy_shm();

  //恢复SIGSEGV信号
  signal(SIGSEGV, SIG_DFL);
}

void ctrl_c_handler(int32_t sig)
{
  PRINT_WARNING("ctrl c handler");

  process_exit(&driver_status);
  exit(EXIT_SUCCESS);
}

DriverConfig load_config(int argv, char **argc)
{
  DriverConfig config = default_driver_config;
  driver_config.is_valid = true;
  bool is_config_file_complete = true;

  if (argv == 1)
    return config;

  const char *filename = argc[1];
  pugi::xml_document doc;
  if (!doc.load_file(filename)) // 0代表 load 成功，没有错误
  {
    config.is_valid = false;
    PRINT_ERROR_FMT("Failed in load xml file : %s !", filename);
    return config;
  }

  pugi::xml_node driver_node = doc.child("DRIVER");
  if (driver_node.empty()) {
    config.is_valid = false;
    PRINT_ERROR("Node empty!");
    return config;
  }

  config.use_imu = driver_node.attribute("use_imu").as_bool();
  config.use_motor = driver_node.attribute("use_motor").as_bool();
  config.use_gps = driver_node.attribute("use_gps").as_bool();
  config.use_lidar = driver_node.attribute("use_lidar").as_bool();
  config.use_soft_interrput =
    driver_node.attribute("use_soft_interrput").as_bool();
  config.wait_for_carto = driver_node.attribute("wait_for_carto").as_bool();

  if (driver_node.attribute("save_data_bag"))
    config.save_data_bag = driver_node.attribute("save_data_bag").as_bool();
  else {
    is_config_file_complete = false;
    config.save_data_bag = false;
  }

  if (driver_node.attribute("load_data_bag")) {
    config.load_data_bag = driver_node.attribute("load_data_bag").as_bool();
    if (config.load_data_bag) {
      config.use_soft_interrput = true;
      if (driver_node.attribute("load_bag_percentage"))
        config.load_bag_percentage =
          driver_node.attribute("load_bag_percentage").as_int();
      else
        config.load_bag_percentage = 100;
    }
  } else {
    is_config_file_complete = false;
    config.load_data_bag = false;
  }

  if (config.load_data_bag && config.save_data_bag) {
    config.is_valid = false;
    PRINT_ERROR("driver connot save and load the bag at the same time!");
    return config;
  }

  config.imu_dev_path = driver_node.attribute("imu_dev_path").as_string();
  config.gps_dev_path = driver_node.attribute("gps_dev_path").as_string();
  config.motor_dev_path1 = driver_node.attribute("motor_dev_path1").as_string();
  config.motor_dev_path2 = driver_node.attribute("motor_dev_path2").as_string();

  if (driver_node.attribute("data_bag_filaname"))
    config.data_bag_filaname =
      driver_node.attribute("data_bag_filaname").as_string();
  else
    is_config_file_complete = false;

  if (driver_node.attribute("load_bag_filename"))
    config.load_bag_filename =
      driver_node.attribute("load_bag_filename").as_string();
  else
    is_config_file_complete = false;

  if (driver_node.attribute("imu_update_freq")) {
    config.imu_update_freq = driver_node.attribute("imu_update_freq").as_int();
    if (config.imu_update_freq > 100)
      config.imu_update_freq = 100;
    else if (config.imu_update_freq < 20)
      config.imu_update_freq = 20;
  } else
    is_config_file_complete = false;

  if (driver_node.attribute("motor_update_freq"))
    config.motor_update_freq =
      driver_node.attribute("motor_update_freq").as_int();
  else
    is_config_file_complete = false;

  if (config.load_data_bag && config.load_bag_filename.empty()) {
    config.is_valid = false;
    PRINT_ERROR("driver needs a data bag!");
    return config;
  }

  config.max_secs_waiting_cmd =
    driver_node.attribute("max_secs_waiting_cmd").as_int();

  const char *imu_types[kImuTypeCount] = { "lpms", "mpu9250" };
  std::string str_imu_type = driver_node.attribute("imu_type").as_string();
  for (int32_t i = 0; i < (int32_t)kImuTypeCount; ++i) {
    if (str_imu_type.compare(imu_types[i]) == 0) {
      config.imu_type = (ImuType)i;
      break;
    }
  }

  // main lidar
  const char *lidar_types[kLidarTypeCount] = { "lslidar_n301", "sick_lms1xx",
                                               "robosense_16", "" };
  std::string str_main_lidar_type =
    driver_node.attribute("main_lidar_type").as_string();
  if (str_main_lidar_type.empty()) {
    config.is_valid = false;
    PRINT_ERROR("the main lidar type can not be empty!");
    return config;
  }
  for (int32_t i = 0; i < (int32_t)kLidarTypeCount - 1; ++i) {
    if (str_main_lidar_type.compare(lidar_types[i]) == 0) {
      config.main_lidar_type = (LidarType)i;
      break;
    }
  }
  // and the other lidar
  std::string str_the_other_lidar_type =
    driver_node.attribute("the_other_lidar_type").as_string();
  if (str_the_other_lidar_type.empty()) {
    PRINT_INFO("the other lidar type is empty.");
    config.the_other_lidar_type = kNoLidar;
  } else {
    for (int32_t i = 0; i < (int32_t)kLidarTypeCount - 1; ++i) {
      if (str_the_other_lidar_type.compare(lidar_types[i]) == 0) {
        config.the_other_lidar_type = (LidarType)i;
        break;
      }
    }
  }

  if (config.main_lidar_type == kSickLMS1xx
      || config.the_other_lidar_type == kSickLMS1xx) {
    PRINT_DEBUG("read sick lmx1xx config!");
    pugi::xml_node sick_node = driver_node.child("SickLms1xx");
    if (sick_node) {
      config.sick_config.ip = sick_node.attribute("ip").as_string();
      config.sick_config.port = sick_node.attribute("port").as_int();
    } else {
      config.is_valid = false;
      PRINT_ERROR("your driver config file is not complete!");
      return config;
    }
  }

  // 如果是 robosense，需要额外读入一些配置
  if (config.main_lidar_type == kRobosense16
      || config.the_other_lidar_type == kRobosense16) {
    PRINT_DEBUG("read robosense 16 config!");
    pugi::xml_node robosense_node = driver_node.child("RoboSense");
    if (robosense_node) {
      config.rsdriver_config.ip = robosense_node.attribute("ip").as_string();
      config.rsdriver_config.port = robosense_node.attribute("port").as_int();
      config.rsdriver_config.model =
        robosense_node.attribute("model").as_string();
      config.rsdriver_config.rpm = robosense_node.attribute("rpm").as_double();
      config.rsdriver_config.time_offset =
        robosense_node.attribute("time_offset").as_double();
      config.rsdriver_config.frame_id =
        robosense_node.attribute("frame_id").as_string();

      config.rsdriver_rawdata_config.anglePath =
        robosense_node.attribute("angle_path").as_string();
      config.rsdriver_rawdata_config.channelPath =
        robosense_node.attribute("channel_path").as_string();
      config.rsdriver_rawdata_config.curvesPath =
        robosense_node.attribute("curves_path").as_string();
      config.rsdriver_rawdata_config.curvesRatePath =
        robosense_node.attribute("curves_rate_path").as_string();
      config.rsdriver_rawdata_config.model = config.rsdriver_config.model;
    } else {
      config.is_valid = false;
      PRINT_ERROR("your driver config file is not complete!");
      return config;
    }
  }

  const char *motor_types[kMotorTypeCount] = { "maxon", "geek", "double_geek" };
  std::string str_motor_type = driver_node.attribute("motor_type").as_string();
  for (int i = 0; i < (int32_t)kMotorTypeCount; ++i) {
    if (str_motor_type.compare(motor_types[i]) == 0) {
      config.motor_type = (MotorType)i;
      break;
    }
  }

  if (!is_config_file_complete)
    PRINT_WARNING("the config file is not complete, but it still works!");

  return config;
}

int32_t init_signals()
{
  int32_t ret = 0;
  if (signal(SIGSEGV, segment_fault_handler) == SIG_ERR) {
    perror("[Liuyc] signal error: ");
    ret++;
  }
  if (signal(SIGINT, ctrl_c_handler) == SIG_ERR) {
    perror("[Liuyc] signal error: ");
    ret++;
  }

  return ret;
}

int32_t init_sems(void)
{
  if (driver_config.use_imu) {
    local_imu_sem_id = g_ptr_shm->imu_sem_id;
    driver_status.imu_sem_init_finished = local_imu_sem_id > 0 ? TRUE : FALSE;
    if (driver_status.imu_sem_init_finished == FALSE)
      return BGS_ERROR;

    PRINT_INFO("imu semaphore init succeed!");
  } else {
    local_imu_sem_id = -1;
    driver_status.imu_sem_init_finished = FALSE;
  }

  if (driver_config.use_gps) {
    LOCK_SHM;
    local_gps_sem_id = g_ptr_shm->gps_sem_id;
    UNLOCK_SHM;
    driver_status.gps_sem_init_finished = local_gps_sem_id > 0 ? TRUE : FALSE;
    if (driver_status.gps_sem_init_finished == FALSE) {
      local_gps_sem_id = -1;
      PRINT_ERROR("gps semaphore init failed.");
      return BGS_ERROR;
    }
    PRINT_INFO("gps semaphore init succeed!");
  } else {
    local_gps_sem_id = -1;
    driver_status.gps_sem_init_finished = FALSE;
  }

  PRINT_DEBUG_FMT("gps sem id = %d", local_gps_sem_id);
  if(driver_config.use_lidar) 
  {
    LOCK_SHM;
    local_lidar_sem_id = g_ptr_shm->lidar_sem_id;
    UNLOCK_SHM;
    driver_status.lidar_sem_init_finished = local_lidar_sem_id > 0 ? TRUE : FALSE;
    if (driver_status.lidar_sem_init_finished == FALSE) {
      local_lidar_sem_id = -1;
      PRINT_ERROR("lidar semaphore init failed.");
      return BGS_ERROR;
    }
    PRINT_INFO("lidar semaphore init succeed!");
  } else {
    local_lidar_sem_id = -1;
    driver_status.lidar_sem_init_finished = FALSE;
  }

  PRINT_DEBUG_FMT("lidar sem id = %d", local_lidar_sem_id);

  return BGS_OK;
}


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
    g_ptr_shm->error.lidar_error = TRUE;
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
      return BGS_OK;
    }
    else {
      PRINT_ERROR("failed to init lidar!");
      main_lidar_collector->DeInit();
      return BGS_ERROR;
    }

    // main_lidar_collector->SetCallback([](laser_scan_msg_ptr)
    // {
    //   if (driver_config.wait_for_carto && g_ptr_shm->n_carto_pid == 0)
    //     return 1;

    //   LOCK_SHM;
    //   g_ptr_shm->scan_msg = *laser_scan_msg_ptr;
    //   UNLOCK_SHM;

    //   sem_post(&lidar_sem);
    //   return 0;
    // });
    break;

  case kRobosense16:

    point_cloud_ptr = boost::make_shared<PointCloud2Msg>();
    point_cloud_ptr->header.frame_id = lidar_frame_id;
    main_lidar_collector = std::make_shared<RoboSenseCollector>(
      driver_config.rsdriver_config, driver_config.rsdriver_rawdata_config,
      point_cloud_ptr, []() {
        if (driver_config.wait_for_carto && g_ptr_shm->n_carto_pid == 0)
          return 1;

        LOCK_SHM;
        g_ptr_shm->point_cloud2_msg = *point_cloud_ptr;
        UNLOCK_SHM;

        sem_post(&lidar_sem);
        return 0;
      });

    if (!main_lidar_collector) {
      PRINT_ERROR("Lidar construct failed!");
      return BGS_ERROR;
    }

    driver_status.main_lidar_ready = TRUE;
    break;

  case kNoLidar:
  default:

    PRINT_ERROR_FMT("Unkown lidar type: %d", driver_config.main_lidar_type);
    return BGS_ERROR;
  }

  if (main_lidar_collector)
    main_lidar_collector->on_error_ = error_func;

  const std::string the_other_frame_id = "the_other_laser_link";
  switch (driver_config.the_other_lidar_type) {
  case kLslidarN301:
    break;

  case kSickLMS1xx:

    if (laser_scan_msg_ptr || the_other_lidar_collector) {
      PRINT_ERROR("there should not be another 2d lidar!");
      return BGS_ERROR;
    }

    laser_scan_msg_ptr = std::make_shared<LaserScanMsg>();
    laser_scan_msg_ptr->header.frame_id = the_other_frame_id;
    the_other_lidar_collector =std::make_shared<LMS1Collector>();

    // (laser_scan_msg_ptr, []() {
    //     LOCK_SHM;
    //     g_ptr_shm->scan_msg = *laser_scan_msg_ptr;
    //     UNLOCK_SHM;
    //     return 0;
    //   });
    if (!the_other_lidar_collector) {
      PRINT_ERROR("Lidar construct failed!");
      return BGS_ERROR;
    }

    if (the_other_lidar_collector->Init(
          (char *)(driver_config.sick_config.ip.c_str()),
          driver_config.sick_config.port)
        == BGS_OK)
      driver_status.the_other_lidar_ready = TRUE;
    else {
      PRINT_ERROR("failed to init lidar!");
      the_other_lidar_collector->DeInit();
      return BGS_ERROR;
    }

    break;

  case kRobosense16:

    if (point_cloud_ptr || the_other_lidar_collector) {
      PRINT_ERROR("there should not be another 3d lidar!");
      return BGS_ERROR;
    }

    point_cloud_ptr = boost::make_shared<PointCloud2Msg>();
    point_cloud_ptr->header.frame_id = the_other_frame_id;
    the_other_lidar_collector = std::make_shared<RoboSenseCollector>(
      driver_config.rsdriver_config, driver_config.rsdriver_rawdata_config,
      point_cloud_ptr, []() {
        LOCK_SHM;
        g_ptr_shm->point_cloud2_msg = *point_cloud_ptr;
        UNLOCK_SHM;
        return 0;
      });
    if (!the_other_lidar_collector) {
      PRINT_ERROR("Lidar construct failed!");
      return BGS_ERROR;
    }

    driver_status.the_other_lidar_ready = TRUE;
    break;

  case kNoLidar:
    break;
  default:

    PRINT_ERROR_FMT("Unkown lidar type: %d", driver_config.main_lidar_type);
    return BGS_ERROR;
  }

  if (the_other_lidar_collector)
    the_other_lidar_collector->on_error_ = error_func;

  PRINT_INFO("lidar init succeed!");
  return BGS_OK;
}

void *lidar_data_update(void *)
{
  if (!main_lidar_collector || !driver_status.main_lidar_ready
      || !driver_status.lidar_sem_init_finished) {
    pthread_exit(NULL);
  }

  if (driver_config.main_lidar_type == kSickLMS1xx) {
    PRINT_INFO("It is a lms1xx sensor");
    driver_status.lidar_thread_running = TRUE;
    int32_t error_count = 0;
    const int32_t max_error_count = 20;
    const int32_t sleep_time_us = 10;
    while (b_process_running) {
      if (driver_config.wait_for_carto && g_ptr_shm->n_carto_pid == 0) {
        usleep(sleep_time_us);
        continue;
      }
      if (main_lidar_collector->GetScanMsg(laser_scan_msg_ptr.get()) == BGS_ERROR)
      {
        error_count++;
        if (error_count >= max_error_count)
          break;
        else {
          usleep(sleep_time_us);
          continue;
        }
      }
      // write shared memory
      LOCK_SHM;
      g_ptr_shm->scan_msg = *laser_scan_msg_ptr;

      // memory barrier
      // 用来防止编译器优化导致的不写内存的问题
      // 类似内核的 mb() wmb()
      BGS_MEM_BARRIER;
      UNLOCK_SHM;

      if (driver_config.wait_for_carto)
        semaphore_v(local_lidar_sem_id);

    //    usleep( sleep_time_us );
    }
  }

  PRINT_INFO("lidar thread exit!");
  driver_status.lidar_thread_running = FALSE;

  pthread_exit(NULL);
}
std::vector<ImuMsgInShm> imus_load_from_bag;
std::vector<AvailableSensors> all_sensor_load_from_bag;
