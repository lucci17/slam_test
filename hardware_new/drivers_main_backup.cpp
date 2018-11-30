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

// threads
pthread_t load_bag_thread = 0;
pthread_t save_bag_thread = 0;
pthread_t soft_interrupt_thread = 0;
pthread_t motor_thread = 0;
pthread_t imu_thread = 0;

DriverConfig default_driver_config = {.is_valid = true,
                                      .use_imu = true,
                                      .use_motor = true,
                                      .use_gps = true,
                                      .use_lidar = true,
                                      .use_soft_interrput = false,
                                      .save_data_bag = false,
                                      .load_data_bag = false,
                                      .wait_for_carto = true,
                                      .main_lidar_type = kNoLidar,
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

  // parameters for loading bag
  // step1 init imu and lidar ... all hard drivers if not in simulation
  if (!driver_config.load_data_bag) // usual mode ( not load bag )
  {
    if (init_imu() == BGS_ERROR || init_gps() == BGS_ERROR
        || init_lidar() == BGS_ERROR || init_motor() == BGS_ERROR) {
      process_exit(&driver_status);
      return -1;
    }

    // lidar thread
    driver_status.soft_interrupt_thread_init_finished = FALSE;
    driver_status.soft_interrupt_thread_running = FALSE;
    if (driver_config.use_soft_interrput) {
      if (pthread_create(&soft_interrupt_thread, NULL, soft_interrupt, NULL)) {
        PRINT_ERROR("Create soft interrupt thread failed!");
        process_exit(&driver_status);
        return -1;
      }
      driver_status.soft_interrupt_thread_init_finished = TRUE;
    }

    // motor thread
    // init motor thread
    // load bag 的情况是无法使用 motor 的，因为motor的控制量并不是由driver给出的
    driver_status.motor_thread_init_finished = FALSE;
    driver_status.motor_thread_running = FALSE;
    if (driver_config.use_motor) {
      if (pthread_create(&motor_thread, NULL, motor_data_update, NULL)) {
        driver_status.motor_thread_init_finished = FALSE;
        PRINT_ERROR("Create motor thread failed!");

        process_exit(&driver_status);
        return -1;
      }
      driver_status.motor_thread_init_finished = TRUE;
    }

    // init imu thread
    driver_status.imu_thread_running = FALSE;
    driver_status.imu_thread_init_finished = FALSE;
    if (driver_config.use_imu) {
      if (pthread_create(&imu_thread, NULL, imu_data_update, NULL)) {
        PRINT_ERROR("Create imu thread failed!");
        process_exit(&driver_status);
        return -1;
      }
      driver_status.imu_thread_init_finished = TRUE;
    }

    // init save bag thread
    // save bag 和 load bag 不可能同时进行
    if (driver_config.save_data_bag
        && !driver_config.data_bag_filaname.empty()) {
      local_save_bag_sem_id = semget(BGS_SAVE_BAG_SEM_KEY, 1, 0666 | IPC_CREAT);
      if (local_save_bag_sem_id == -1
          || init_sem(local_save_bag_sem_id, 0) < 0) {
        // TODO
        driver_status.save_bag_sem_init_finished = FALSE;
        PRINT_ERROR("init save bag sem failed!");
        process_exit(&driver_status);
        return -1;
      }
      driver_status.save_bag_sem_init_finished = TRUE;

      if (pthread_create(&save_bag_thread, NULL, data_bag_update, NULL)) {
        driver_status.save_bag_thread_init_finished = FALSE;
        PRINT_ERROR("Create save bag thread failed!\n");

        process_exit(&driver_status);
        return -1;
      }
      driver_status.save_bag_thread_init_finished = TRUE;
    } else {
      driver_status.save_bag_sem_init_finished = FALSE;
      driver_status.save_bag_thread_init_finished = FALSE;
      driver_status.save_bag_thread_running = FALSE;
    }

  } else // load bag
  {
    // load bag thread
    if (pthread_create(&load_bag_thread, NULL, load_bag_data, NULL)) {
      driver_status.load_bag_thread_init_finished = FALSE;
      PRINT_ERROR("Create load bag thread failed!");
      process_exit(&driver_status);
      return -1;
    }
    driver_status.load_bag_thread_init_finished = TRUE;
  }

  // 线程间同步，且初始化为 0
  // 这样初始化的信号量在进程结束时会自动释放
  driver_status.lidar_sem_init_finished = FALSE;
  if (sem_init(&lidar_sem, 0, 0) != 0) {
    PRINT_ERROR("lidar sem init failed!");
    process_exit(&driver_status);
    return -1;
  }
  driver_status.lidar_sem_init_finished = TRUE;

  g_ptr_shm->driver_status.all_ready = TRUE;

  // step2 get sensor data and write them to the shared memory
  int32_t sample_count = 200;
  int32_t i = 0;
  struct timespec ts;
  int32_t sem_error_count = 0;
  while (b_process_running) {
    // step1 等待信号量（来自 lidar thread）
    // 超时则判断 lidar thread 是否还在运行，如果没有运行则直接退出
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 1; // 超时1秒
    if (sem_timedwait(&lidar_sem, &ts) != 0)
      continue;

    if (driver_status.save_bag_thread_running) {
      static int32_t save_bag_sem_error_count = 0;
      if (semaphore_v(local_save_bag_sem_id) < 0) {
        save_bag_sem_error_count++;
        if (save_bag_sem_error_count >= 20) {
          PRINT_ERROR("sem error for several times");
        }
      }
    }

    // 信号量多次出错之后直接停止 driver 的运行
    // 暂定为 20 次
    if (driver_config.wait_for_carto && semaphore_v(local_carto_sem_id) < 0) {
      sem_error_count++;
      if (sem_error_count >= 20) {
        PRINT_ERROR("sem error for several times");
        break;
      }
    }

    if (driver_config.wait_for_carto && driver_config.use_gps
        && semaphore_v(local_gps_sem_id) < 0) {
      sem_error_count++;
      if (sem_error_count >= 20) {
        PRINT_ERROR("sem error for several times");
        break;
      }
    }

    i++;
    if (!driver_config.load_data_bag && i >= sample_count) {
      PRINT_DEBUG_FMT("lidar recieved for %d times!", sample_count);
      i = 0;
    }
  }

  // step3 finish process
  process_exit(&driver_status);
  PRINT_INFO("Main Thread quit!");
  return 0;
}

void *imu_data_update(void *)
{
  if (!imu_collector || !driver_status.imu_ready
      || !driver_status.imu_sem_init_finished) {
    pthread_exit(NULL);
  }

  if (driver_config.imu_type == kLpms) {
    PRINT_INFO("It is a lpms sensor, no need to init a thread for it.");
    dynamic_cast<LpmsSensor *>(imu_collector.get())
      ->set_callback(lpms_callback);
    driver_status.imu_thread_running = FALSE;
    pthread_exit(NULL);
  }

  driver_status.imu_thread_running = TRUE;
  int32_t error_count = 0;
  const int32_t max_error_count = 200;
  const int32_t sleep_time_us = 1000000 / driver_config.imu_update_freq;
  while (b_process_running) {
    if (driver_config.wait_for_carto && g_ptr_shm->n_carto_pid == 0) {
      usleep(sleep_time_us);
      continue;
    }
    if (imu_collector->get_imu_msg(imu_msg_ptr.get()) == BGS_ERROR
        || !imu_msg_valid(imu_msg_ptr)) {
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
    g_ptr_shm->imu_msg = *imu_msg_ptr;

    // memory barrier
    // 用来防止编译器优化导致的不写内存的问题
    // 类似内核的 mb() wmb()
    BGS_MEM_BARRIER;
    UNLOCK_SHM;

    if (driver_config.save_data_bag) {
      pthread_mutex_lock(&mutex_for_imu_vec);
      saved_imu_for_bag.push_back(g_ptr_shm->imu_msg);
      pthread_mutex_unlock(&mutex_for_imu_vec);
    }

    if (driver_config.wait_for_carto)
      semaphore_v(local_imu_sem_id);

    // 		usleep( sleep_time_us );
  }

  PRINT_INFO("imu thread exit!");
  driver_status.imu_thread_running = FALSE;

  pthread_exit(NULL);
}

void process_exit(DriverStatus *status)
{
  b_process_running = false;
  kill_soft_interrupt_thread();
  kill_motor_thread();
  kill_save_bag_thread();
  kill_imu_thread();
  kill_load_bag_thread();

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
    main_lidar_collector->deInit();
  }
  if (the_other_lidar_collector && status->the_other_lidar_ready) {
    PRINT_DEBUG("deinit the other lidar");
    the_other_lidar_collector->deInit();
  }
  if (motor && status->motor_ready) {
    PRINT_DEBUG("deinit motor");
    motor->deInit();
  }

  if (status->shm_init_finished)
    destroy_shm();
}

void kill_soft_interrupt_thread()
{
  if (driver_status.soft_interrupt_thread_init_finished
      && soft_interrupt_thread != 0)
    pthread_join(soft_interrupt_thread, NULL);
  driver_status.soft_interrupt_thread_init_finished = FALSE;

  if (driver_status.lidar_sem_init_finished) {
    sem_destroy(&lidar_sem);
    driver_status.lidar_sem_init_finished = FALSE;
  }
}

void kill_save_bag_thread()
{
  if (driver_status.save_bag_thread_running && save_bag_thread != 0)
    pthread_join(save_bag_thread, NULL);
  driver_status.save_bag_thread_init_finished = false;

  if (driver_status.save_bag_sem_init_finished) {
    del_semvalue(local_save_bag_sem_id);
    driver_status.save_bag_sem_init_finished = FALSE;
  }
}

void kill_load_bag_thread()
{
  if (driver_status.load_bag_thread_init_finished && load_bag_thread != 0)
    pthread_join(load_bag_thread, NULL);

  driver_status.load_bag_thread_init_finished = FALSE;
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

void *soft_interrupt(void *)
{
  driver_status.soft_interrupt_thread_running = TRUE;
  g_ptr_shm->driver_status.lidar_thread_running = TRUE;
  // 	BgsTime time_delta;
  // 	BgsTime time_for_sleep;
  // 	// 这个变量用于滤除掉时间戳相同的数据
  // 	// 认为相差不到 1ms 的数据为相同数据，舍弃掉
  // 	const BgsTime min_time_sleep( 0, 1000000 );	// 1ms

  // first: available_sensors_
  // second: sensor data ( including time stamp )
  bool b_wait = true;
  while (b_process_running) {
    b_wait = (driver_config.wait_for_carto && g_ptr_shm->n_carto_pid == 0);
    // 虚拟的一个雷达中断
    if (b_wait) {
      msleep(10);
      continue;
    }

    msleep(100);
    sem_post(&lidar_sem);
  }

  PRINT_INFO("Lidar Thread quit!");
  driver_status.soft_interrupt_thread_running = FALSE;
  g_ptr_shm->driver_status.lidar_thread_running = FALSE;
  pthread_exit(NULL);
}

void kill_motor_thread()
{
  if (driver_status.motor_thread_init_finished && motor_thread != 0)
    pthread_join(motor_thread, NULL);
  driver_status.motor_thread_init_finished = FALSE;
}

void kill_imu_thread()
{
  if (driver_status.imu_thread_init_finished && imu_thread != 0)
    pthread_join(imu_thread, NULL);
  driver_status.imu_thread_init_finished = FALSE;
}

#define IS_NEW_CMD 1
#define NOT_NEW_CMD 0
void *motor_data_update(void *)
{
  if (!motor)
    pthread_exit(NULL);

  driver_status.motor_thread_running = TRUE;
  static double start_time_sec = BgsTime::get_current_time().toSec();
  double current_time_sec = 0.;
  double max_wait_time_sec = driver_config.max_secs_waiting_cmd < 0
                               ? 0.d
                               : driver_config.max_secs_waiting_cmd;

  const int32_t time_sleep_us = 1000000 / driver_config.motor_update_freq;
  while (b_process_running) {
    // get data from share memory
    LOCK_SHM;
    *motor_msg_ptr = g_ptr_shm->motor_msg;
    UNLOCK_SHM;

    current_time_sec = BgsTime::get_current_time().toSec();
    if (current_time_sec + 1.e-6 < start_time_sec) {
      PRINT_ERROR("get time error!");
      break;
    }

    if (motor_msg_ptr->is_new_command == IS_NEW_CMD) {
      start_time_sec = current_time_sec;
      motor_msg_ptr->d_wait_secs = 0.;
    } else {
      motor_msg_ptr->d_wait_secs = current_time_sec - start_time_sec;
    }

    // 超时保护，如果连续若干秒			// TODO add
    // flags都没有获取新的速度指令，则让机器停下来
    if (motor_msg_ptr->d_wait_secs < max_wait_time_sec) {
      motor->setSpeed(motor_msg_ptr->motor_speedL, motor_msg_ptr->motor_speedR);
    } else {
      motor_msg_ptr->motor_speedL = 0;
      motor_msg_ptr->motor_speedR = 0;
    }

    motor_msg_ptr->motor_pL = motor_msg_ptr->motor_cL;
    motor_msg_ptr->motor_pR = motor_msg_ptr->motor_cR;
    // motor->setSpeed( motor_msg.motor_speedL, motor_msg.motor_speedR );
    // motor->getSpeed(motor_msg_ptr->motor_cL, motor_msg_ptr->motor_cR);
    motor_msg_ptr->motor_cL = motor_msg_ptr->motor_speedL;
    motor_msg_ptr->motor_cR = motor_msg_ptr->motor_speedR;
    motor_msg_ptr->header.stamp = BgsTime::get_current_time();
    motor_msg_ptr->is_new_command = NOT_NEW_CMD;

    LOCK_SHM;
    g_ptr_shm->motor_msg = *motor_msg_ptr;
    UNLOCK_SHM;
    usleep(time_sleep_us);
  }

  PRINT_INFO("Motor Thread quit!");
  driver_status.motor_thread_running = FALSE;
  driver_status.motor_thread_init_finished = FALSE;
  pthread_exit(NULL);
}

void *data_bag_update(void *)
{
  std::ofstream fout_save_data;
  driver_status.save_bag_thread_running = TRUE;
  fout_save_data.open(driver_config.data_bag_filaname, std::ios::binary);

  // step 1
  // write the start frame
  DataFrame data_frame;
  StartData start_data;
  start_data.stamp_ = BgsTime::get_current_time();
  start_data.all_available_sensors_ = 0;
  if (driver_config.use_lidar && driver_status.main_lidar_ready) {
    if (main_lidar_collector->type_ == _3D_LIDAR_)
      start_data.all_available_sensors_ |= BAG_POINT_CLOUD_AVAILABLE;
    else
      start_data.all_available_sensors_ |= BAG_LASER_AVAILABLE;
  }
  if (driver_config.use_imu)
    start_data.all_available_sensors_ |= BAG_IMU_AVAILABLE;
  if (driver_config.use_gps)
    start_data.all_available_sensors_ |= BAG_GPS_AVAILABLE;
  if (driver_config.use_motor)
    start_data.all_available_sensors_ |= BAG_MTR_AVAILABLE;

  // TODO odom
  data_frame.type_ = hardware::databag::kStartFrame;
  data_frame.data_ = (char *)(&start_data);
  if (write_to_data_bag(fout_save_data, data_frame) == -1) {
    fout_save_data.close();
    driver_status.save_bag_thread_running = FALSE;
    return NULL;
  }

  // step 2
  // write the sensor data
  struct timespec data_wait_time;
  data_wait_time.tv_sec = 0;
  data_wait_time.tv_nsec = BGS_100MS * 5;

  const int32_t size_of_shm = sizeof(GlobalShm);
  uLongf size_of_data = size_of_shm * 2;
  Byte compressed_data[size_of_data];
  SensorData sensor_data;
  int32_t sem_error_count = 0;
  int32_t frame_count = 0;
  data_frame.data_ = (char *)(&sensor_data);
  while (b_process_running) {
    // revieve semaphore
    int ret = semaphore_p_timed(local_save_bag_sem_id, &data_wait_time);
    if (ret < 0) {
      PRINT_ERROR("sem p error!");
      sem_error_count++;
      if (sem_error_count > 20) {
        PRINT_ERROR("save bag sem error for several times!");
        break;
      }
    } else if (ret > 0) {
      // time out
      continue;
    }

    // 先获取所有的未被保存的数据
    std::vector<ImuMsgInShm> imus;
    pthread_mutex_lock(&mutex_for_imu_vec);
    imus.swap(saved_imu_for_bag);
    pthread_mutex_unlock(&mutex_for_imu_vec);

    // imu
    uLongf size_of_imu = sizeof(ImuMsgInShm) * 2;
    Byte compressed_data_imu[size_of_imu];
    // global ( all sensors )
    GlobalShm data;
    LOCK_SHM;
    BGS_MEM_BARRIER;
    memcpy(&data, g_ptr_shm, sizeof(data));
    UNLOCK_SHM;

    memset(compressed_data, 0, size_of_data);
    BgsTime lidar_time;
    if (main_lidar_collector->type_ == _3D_LIDAR_)
      lidar_time = data.point_cloud2_msg.header.stamp;
    else
      lidar_time = data.scan_msg.header.stamp;

    bool all_sensor_finished = false;
    // 压缩并保存
    for (auto &imu : imus) {
      memset(compressed_data_imu, 0, size_of_imu);
      if (lidar_time.toSec() - imu.header.stamp.toSec() > 0.001d) {
        data_frame.type_ = hardware::databag::kOnlyImuFrame;
        sensor_data.compressed_size_ = size_of_imu;
        sensor_data.available_sensors_ = BAG_IMU_AVAILABLE;
        if (compress(compressed_data_imu, &(sensor_data.compressed_size_),
                     (Byte *)(&imu), sizeof(ImuMsgInShm))
            == Z_OK) {
          sensor_data.compressed_data_ = compressed_data_imu;
          write_to_data_bag(fout_save_data, data_frame);
          frame_count++;
        }
      } else if (fabs(lidar_time.toSec() - imu.header.stamp.toSec())
                 <= 0.001d) {
        if (all_sensor_finished) {
          data_frame.type_ = hardware::databag::kOnlyImuFrame;
          sensor_data.compressed_size_ = size_of_imu;
          sensor_data.available_sensors_ = BAG_IMU_AVAILABLE;
          if (compress(compressed_data_imu, &(sensor_data.compressed_size_),
                       (Byte *)(&imu), sizeof(ImuMsgInShm))
              == Z_OK) {
            sensor_data.compressed_data_ = compressed_data_imu;
            write_to_data_bag(fout_save_data, data_frame);
            frame_count++;
          }
        } else {
          data.imu_msg = imu;
          sensor_data.compressed_size_ = size_of_data;
          sensor_data.available_sensors_ = start_data.all_available_sensors_;
          data_frame.type_ = hardware::databag::kAllSensorFrame;
          if (compress(compressed_data, &(sensor_data.compressed_size_),
                       (Byte *)(&data), size_of_shm)
              == Z_OK) {
            sensor_data.compressed_data_ = compressed_data;
            write_to_data_bag(fout_save_data, data_frame);
            frame_count++;

            all_sensor_finished = true;
          } else {
            sensor_data.compressed_size_ = 0;
            PRINT_DEBUG("compress failed!");
          }
        }
      } else if (lidar_time.toSec() - imu.header.stamp.toSec() < -0.001d) {
        if (!all_sensor_finished) {
          sensor_data.compressed_size_ = size_of_data;
          sensor_data.available_sensors_ =
            (start_data.all_available_sensors_
             & (~BAG_IMU_AVAILABLE)); // 剔除掉数据里面的IMU部分
          data_frame.type_ = hardware::databag::kAllSensorFrame;
          if (compress(compressed_data, &(sensor_data.compressed_size_),
                       (Byte *)(&data), size_of_shm)
              == Z_OK) {
            sensor_data.compressed_data_ = compressed_data;
            write_to_data_bag(fout_save_data, data_frame);
            frame_count++;
            all_sensor_finished = true;
          } else {
            sensor_data.compressed_size_ = 0;
            PRINT_DEBUG("compress failed!");
          }
        }

        data_frame.type_ = hardware::databag::kOnlyImuFrame;
        sensor_data.compressed_size_ = size_of_imu;
        sensor_data.available_sensors_ = BAG_IMU_AVAILABLE;
        if (compress(compressed_data_imu, &(sensor_data.compressed_size_),
                     (Byte *)(&imu), sizeof(ImuMsgInShm))
            == Z_OK) {
          sensor_data.compressed_data_ = compressed_data_imu;
          write_to_data_bag(fout_save_data, data_frame);
          frame_count++;
        }
      }
    }

    // 给共享内存上锁后，copy出共享内存的内容再压缩
    // 压缩是很耗时的工作，如果在内存锁的情况下进行压缩可能会造成堵塞的情况
    if (!all_sensor_finished) {
      sensor_data.compressed_size_ = size_of_data;
      sensor_data.available_sensors_ =
        (start_data.all_available_sensors_
         & (~BAG_IMU_AVAILABLE)); // 剔除掉数据里面的IMU部分
      data_frame.type_ = hardware::databag::kAllSensorFrame;
      if (compress(compressed_data, &(sensor_data.compressed_size_),
                   (Byte *)(&data), size_of_shm)
          == Z_OK) {
        sensor_data.compressed_data_ = compressed_data;
        write_to_data_bag(fout_save_data, data_frame);
        frame_count++;
        all_sensor_finished = true;
      } else {
        sensor_data.compressed_size_ = 0;
        PRINT_DEBUG("compress failed!");
      }
    }
  }

  // step 3
  // write the end frame
  data_frame.type_ = hardware::databag::kEndFrame;
  EndData end_data;
  data_frame.data_ = (char *)(&end_data);
  end_data.all_frame_count = frame_count;
  write_to_data_bag(fout_save_data, data_frame);

  fout_save_data.close();
  driver_status.save_bag_thread_running = FALSE;
  pthread_exit(NULL);
}

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

  const char *motor_types[kMotorTypeCount] = { "maxon", "geek", "double_geek", "copley" };
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

  return BGS_OK;
}

bool imu_msg_valid(const Bgs::ImuMsgPtr &imu_msg)
{
  bool b_valid = true;
  double entire_acc =
    sqrt(imu_msg->linear_acceleration.x * imu_msg->linear_acceleration.x
         + imu_msg->linear_acceleration.y * imu_msg->linear_acceleration.y
         + imu_msg->linear_acceleration.z * imu_msg->linear_acceleration.z);

  if (entire_acc < 1.0e-1 || entire_acc > 1.0e2) {
    PRINT_WARNING_FMT("acc error: %lf", entire_acc);
    b_valid = false;
  }

  return b_valid;
}

int32_t init_motor(void)
{
  driver_status.motor_ready = FALSE;
  if (!driver_config.use_motor)
    return BGS_OK;

  switch (driver_config.motor_type) {
  case kMaxon:
    break;

  case kGeek:
    motor = std::make_shared<GeekMotor>();
    if (motor->initialisation(driver_config.motor_dev_path1.c_str())
        == BGS_ERROR) {
      return BGS_ERROR;
    }
    driver_status.motor_ready = TRUE;
    break;

  case kDoubleGeek:
    motor = std::make_shared<DoubleGeekMotor>();
    if (!motor) {
      PRINT_ERROR("Motor construct failed!");
      return BGS_ERROR;
    }
    if (motor->initialisation(driver_config.motor_dev_path1.c_str(),
                              driver_config.motor_dev_path2.c_str())
        == BGS_ERROR) {
      return BGS_ERROR;
    }
    driver_status.motor_ready = TRUE;

    break;

  case kCopley:
    motor = std::make_shared<CopleyMotor>();
    if (!motor) {
      PRINT_ERROR("Motor construct failed!");
      return BGS_ERROR;
    }

    if (motor->initialisation(driver_config.motor_dev_path1.c_str())
        == BGS_ERROR) {
      return BGS_ERROR;
    }
    driver_status.motor_ready = TRUE;
    break;

  default:
    break;
  }

  motor_msg_ptr = boost::make_shared<MotorMsg>();
  motor_msg_ptr->header.frame_id = "differential_motor_link";
  motor_msg_ptr->motor_pL = 0;
  motor_msg_ptr->motor_pR = 0;
  motor_msg_ptr->motor_cL = 0;
  motor_msg_ptr->motor_cR = 0;
  motor_msg_ptr->motor_speedL = 0;
  motor_msg_ptr->motor_speedR = 0;
  return BGS_OK;
}

int32_t init_gps(void)
{
  driver_status.gps_ready = FALSE;
  if (!driver_config.use_gps)
    return BGS_OK;

  gps_collector = std::make_shared<GPSCollector>();
  if (!gps_collector) {
    PRINT_ERROR("Get GPS error!");
    return BGS_ERROR;
  }
  if (gps_collector->init((char *)driver_config.gps_dev_path.c_str())
      == BGS_ERROR) {
    PRINT_ERROR("GPS: init error !");
    return BGS_ERROR;
  }

  gps_collector->set_callback(
    [](gnss::GPSData current_gps, gnss::GPSGesture current_gst) {
      if (driver_config.wait_for_carto && g_ptr_shm->n_carto_pid == 0)
        return 1;

      GpsMsgPtr gps_msg_ptr = boost::make_shared<GpsMsg>();
      // 			PRINT_DEBUG_FMT("time: %lf", BgsTime::get_current_time().toSec()
      // );
      if(driver_count == 20)
      {
        driver_count = 0;
        PRINT_DEBUG_FMT("gps: mode: %d, position:%lf %lf %lf", current_gps.mode,
                current_gps.enu.x, current_gps.enu.y, current_gps.enu.z);
      }
      driver_count++;
      gps_msg_ptr->header.stamp = BgsTime::get_current_time();

      gps_msg_ptr->gps_position.x = current_gps.enu.x;
      gps_msg_ptr->gps_position.y = current_gps.enu.y;
      gps_msg_ptr->gps_position.z = current_gps.enu.z;
      gps_msg_ptr->gps_pmode = current_gps.mode;
      gps_msg_ptr->gps_heading = current_gst.yaw;
      gps_msg_ptr->gps_gmode = current_gst.mode;
      gps_msg_ptr->has_new_pos = TRUE;

      // then lock the shared_memory
      LOCK_SHM;
      g_ptr_shm->gps_msg = *gps_msg_ptr;
      BGS_MEM_BARRIER;
      UNLOCK_SHM;

      if (driver_status.gps_sem_init_finished && driver_config.wait_for_carto)
        semaphore_v(local_gps_sem_id);

      gps_msg_ptr.reset();
      return 0;
    });

  driver_status.gps_ready = TRUE;
  return BGS_OK;
}

void lpms_callback(ImuData data, const char *id)
{
  if (!b_process_running)
    return;
  if (driver_config.wait_for_carto && g_ptr_shm->n_carto_pid <= 0)
    return;

  hardware::ImuData_to_ImuMsg(data, imu_msg_ptr.get());
  if (!imu_msg_valid(imu_msg_ptr)) {
    PRINT_ERROR("imu msg is not valid.");
    return;
  }

  // write shared memory
  LOCK_SHM;
  g_ptr_shm->imu_msg = *imu_msg_ptr;

  // memory barrier
  // 用来防止编译器优化导致的不写内存的问题
  // 类似内核的 mb() wmb()
  BGS_MEM_BARRIER;
  UNLOCK_SHM;

  if (driver_config.save_data_bag) {
    pthread_mutex_lock(&mutex_for_imu_vec);
    saved_imu_for_bag.push_back(g_ptr_shm->imu_msg);
    pthread_mutex_unlock(&mutex_for_imu_vec);
  }

  if (driver_config.wait_for_carto)
    semaphore_v(local_imu_sem_id);
}

int32_t init_imu(void)
{
  driver_status.imu_ready = FALSE;
  if (!driver_config.use_imu) {
    return BGS_OK;
  }

  switch (driver_config.imu_type) {
  case kLpms:
    imu_collector = std::make_shared<LpmsSensor>();
    if (!imu_collector) {
      PRINT_ERROR("Imu construct failed!");
      return BGS_ERROR;
    }

    imu_collector->init((char *)(driver_config.imu_dev_path.c_str()));
    driver_status.imu_ready = TRUE;
    break;

  case kMPU9250:
    break;

  default:
    driver_status.imu_ready = FALSE;
    PRINT_ERROR_FMT("Unkown imu type: %d", driver_config.imu_type);
    return BGS_ERROR;
    break;
  }

  imu_msg_ptr = boost::make_shared<ImuMsg>();
  msleep(500);
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

    laser_scan_msg_ptr = boost::make_shared<LaserScanMsg>();
    laser_scan_msg_ptr->header.frame_id = lidar_frame_id;
    main_lidar_collector =
      std::make_shared<LMS1Collector>(laser_scan_msg_ptr, []() {
        if (driver_config.wait_for_carto && g_ptr_shm->n_carto_pid == 0)
          return 1;

        LOCK_SHM;
        g_ptr_shm->scan_msg = *laser_scan_msg_ptr;
        UNLOCK_SHM;

        sem_post(&lidar_sem);
        return 0;
      });

    if (!main_lidar_collector) {
      PRINT_ERROR("Lidar construct failed!");
      return BGS_ERROR;
    }
    if (main_lidar_collector->init(
          (char *)(driver_config.sick_config.ip.c_str()),
          driver_config.sick_config.port)
        == BGS_OK)
      driver_status.main_lidar_ready = TRUE;
    else {
      PRINT_ERROR("failed to init lidar!");
      main_lidar_collector->deInit();
      return BGS_ERROR;
    }
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

    laser_scan_msg_ptr = boost::make_shared<LaserScanMsg>();
    laser_scan_msg_ptr->header.frame_id = the_other_frame_id;
    the_other_lidar_collector =
      std::make_shared<LMS1Collector>(laser_scan_msg_ptr, []() {
        LOCK_SHM;
        g_ptr_shm->scan_msg = *laser_scan_msg_ptr;
        UNLOCK_SHM;
        return 0;
      });
    if (!the_other_lidar_collector) {
      PRINT_ERROR("Lidar construct failed!");
      return BGS_ERROR;
    }

    if (the_other_lidar_collector->init(
          (char *)(driver_config.sick_config.ip.c_str()),
          driver_config.sick_config.port)
        == BGS_OK)
      driver_status.the_other_lidar_ready = TRUE;
    else {
      PRINT_ERROR("failed to init lidar!");
      the_other_lidar_collector->deInit();
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

std::vector<ImuMsgInShm> imus_load_from_bag;
std::vector<AvailableSensors> all_sensor_load_from_bag;
void *load_bag_data(void *)
{
  driver_status.load_bag_thread_running = FALSE;

  // 打开这个 bag
  // file，并读取第一帧数据（第一帧数据是开始的数据，带有整个包的一些信息，并不包含传感器数据）
  bag_data = (Byte *)open_bag_file(driver_config.load_bag_filename.c_str(),
                                   &bag_file_size);
  if (bag_data == MAP_FAILED) {
    driver_status.has_loaded_bag = FALSE;
    PRINT_ERROR("open bag file failed!");
    pthread_exit(NULL);
  }
  driver_status.has_loaded_bag = TRUE;

  FrameType type;
  StartData start_data;
  if ((bag_data_offset = read_single_frame(bag_data, bag_data_offset, &type,
                                           &start_data, NULL, NULL))
      != -1) {
    if (type != hardware::databag::kStartFrame) {
      PRINT_ERROR("is should be a start frame!");
      pthread_exit(NULL);
    }

    // 读取 start frame 成功，输出一些信息
    PRINT_INFO("succeed to read start frame!");
  } else {
    PRINT_ERROR("read frame failed!");
    pthread_exit(NULL);
  }

  int32_t all_sensor_frame_count = 0;
  { // 获取整个包的数据个数，用于显示读取百分比
    EndData tmp_end_data;
    int32_t tmp_end_data_offset =
      bag_file_size - (sizeof(DataFrame::start_) + sizeof(DataFrame::end_)
                       + sizeof(EndData) + sizeof(DataFrame::type_));
    FrameType tmp_type;
    if ((tmp_end_data_offset = read_single_frame(
           bag_data, tmp_end_data_offset, &tmp_type, NULL, NULL, &tmp_end_data))
        != -1) {
      if (tmp_type != hardware::databag::kEndFrame) {
        PRINT_ERROR("is should be an end frame!");
        pthread_exit(NULL);
      }

      all_sensor_frame_count = tmp_end_data.all_frame_count;
      PRINT_INFO_FMT("the bag has %d sensor data frames.",
                     all_sensor_frame_count);
    } else {
      PRINT_ERROR("read frame failed!");
      pthread_exit(NULL);
    }
  }

  driver_status.load_bag_thread_running = TRUE;
  const int32_t size_of_shm = sizeof(GlobalShm);
  SensorData sensor_data;

  Byte *uncompressed_data = (Byte *)malloc(size_of_shm * 2);
  sensor_data.uncompressed_data_ = uncompressed_data;
  EndData end_data;

  bool got_time_delta = false;
  bool got_all_data_of_bag = false;
  BgsTime time_delta;

  // first: available_sensors_
  // second: sensor data ( including time stamp )

  bool b_wait = true;
  int32_t current_sensor_frame_index = 0;
  int32_t current_percentage = 0;

  bool have_reset_sem = false;
  while (b_process_running) {
    b_wait = (driver_config.wait_for_carto && g_ptr_shm->n_carto_pid == 0);
    if (b_wait) {
      if (!have_reset_sem) {
        init_sem(local_imu_sem_id, 0);
        have_reset_sem = true;
        PRINT_DEBUG("reset the sem ...");
      }
      msleep(10);
      continue;
    }
    sensor_data.uncompressed_size_ = size_of_shm * 2;

    have_reset_sem = false;
    if (got_time_delta) {
      if (!imus_load_from_bag.empty()
          && imus_load_from_bag.begin()->header.stamp + time_delta
               <= BgsTime::get_current_time()) {
        ImuMsgInShm tmp_imu = imus_load_from_bag.front();
        tmp_imu.header.stamp += time_delta;

        LOCK_SHM;
        g_ptr_shm->imu_msg = tmp_imu;

        BGS_MEM_BARRIER;
        UNLOCK_SHM;

        imus_load_from_bag.erase(imus_load_from_bag.begin());
        if (driver_config.wait_for_carto)
          semaphore_v(local_imu_sem_id);
      }

      if (!all_sensor_load_from_bag.empty()
          && get_time_stamp_of_global(all_sensor_load_from_bag.front())
                 + time_delta
               <= BgsTime::get_current_time()) {
        auto tmp_all = all_sensor_load_from_bag.begin();
        align_time_stamp(*tmp_all, time_delta);
        write_data_to_shm(*tmp_all);

        all_sensor_load_from_bag.erase(tmp_all);

        sem_post(&lidar_sem);
      }

      // 如果这个 vector
      // 的数量不小于2，则暂停读取，一直到有数据发出并从vector剔除之后再继续都文件
      if (all_sensor_load_from_bag.size() >= 2) {
        msleep(1);
        continue;
      }
    }

    if (got_all_data_of_bag) {
      if (all_sensor_load_from_bag.empty() && imus_load_from_bag.empty())
        break;

      msleep(1);
      continue;
    }

    if ((bag_data_offset = read_single_frame(bag_data, bag_data_offset, &type,
                                             NULL, &sensor_data, &end_data))
        != -1) {
      switch (type) {
      case hardware::databag::kStartFrame:
        // start frame 已经读入过了，如果再次读到 start 就说明包有问题
        PRINT_ERROR("should not be a start frame!");
        goto LOAD_BAG_THREAD_EXIT;

      case hardware::databag::kAllSensorFrame:
        // PRINT_DEBUG("Got a Complete All Sensor Frame!");
        if (sensor_data.uncompressed_size_ != sizeof(GlobalShm)) {
          PRINT_ERROR("the sensor frame is incomplete!");
          goto LOAD_BAG_THREAD_EXIT;
        }

        all_sensor_load_from_bag.push_back(
          { sensor_data.available_sensors_,
            *(GlobalShm *)(sensor_data.uncompressed_data_) });
        if (sensor_data.available_sensors_ & BAG_IMU_AVAILABLE)
          imus_load_from_bag.push_back(
            ((GlobalShm *)(sensor_data.uncompressed_data_))->imu_msg);

        if (!got_time_delta && all_sensor_load_from_bag.size() == 2) {
          BgsTime min_time;
          if (!imus_load_from_bag.empty())
            min_time = std::min<BgsTime>(
              imus_load_from_bag.begin()->header.stamp,
              get_time_stamp_of_global(all_sensor_load_from_bag.front()));

          else
            min_time =
              get_time_stamp_of_global(all_sensor_load_from_bag.front());

          time_delta = BgsTime::get_current_time() - min_time;
          got_time_delta = true;
        }

        current_sensor_frame_index++;
        break;

      case hardware::databag::kOnlyImuFrame:
        if (sensor_data.uncompressed_size_ != sizeof(ImuMsgInShm)) {
          PRINT_ERROR("the imu frame is incomplete!");
          goto LOAD_BAG_THREAD_EXIT;
        }

        imus_load_from_bag.push_back(
          *(ImuMsgInShm *)(sensor_data.uncompressed_data_));
        current_sensor_frame_index++;
        break;

      case hardware::databag::kEndFrame:

        if (bag_data_offset == bag_file_size)
          PRINT_INFO("the entire bag has been read!");
        else
          PRINT_WARNING("something wrong when reading bag!");

        got_all_data_of_bag = true;
        break;

      default:
        PRINT_ERROR("not supported frame!");
        goto LOAD_BAG_THREAD_EXIT;
      }

      int32_t tmp_percentage =
        (current_sensor_frame_index == all_sensor_frame_count)
          ? 100
          : (double)current_sensor_frame_index / (double)all_sensor_frame_count
              * 100.;
      if (current_percentage != tmp_percentage) {
        current_percentage = tmp_percentage;
        PRINT_INFO_FMT("read bag ... %d %%", current_percentage);

        if (current_percentage >= driver_config.load_bag_percentage) {
          PRINT_INFO_FMT(
            "arrive the target percentage: %d %%, quit loading bag.",
            driver_config.load_bag_percentage);
          got_all_data_of_bag = true;
        }
      }
    } else // 读取数据失败，直接退出循环（退出线程）
    {
      PRINT_DEBUG("failed to read a single frame!");
      break;
    }
  }

LOAD_BAG_THREAD_EXIT:

  // free all memory
  free(uncompressed_data);
  imus_load_from_bag.clear();
  all_sensor_load_from_bag.clear();

  if (driver_status.has_loaded_bag) {
    close_bag_file(bag_data, bag_file_size);
    driver_status.has_loaded_bag = FALSE;
  }

  driver_status.load_bag_thread_running = FALSE;
  PRINT_INFO("load bag thread exit");

  // load bag 结束后直接结束整个进程
  b_process_running = false;
  pthread_exit(NULL);
}
