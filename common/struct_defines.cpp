#include "struct_defines.h"
#include "conversions.h"

namespace Bgs
{

Header::Header()
    : u_seq_num(0)
    , frame_id("")
{
}

Header &Header::operator=(const HeaderInShm &header_in_shm)
{
  u_seq_num = header_in_shm.u_seq_num;
  stamp = header_in_shm.stamp;
  frame_id = std::string(header_in_shm.s_frame_id);

  return *this;
}

/* TODO 这个函数有BUG，暂时不要使用 */
Header &Header::operator=(const Header &h)
{
  u_seq_num = h.u_seq_num;
  stamp = h.stamp;
  frame_id = h.frame_id;

  return *this;
}

// 在共享内存中，建议使用char数组而不是string来保存字符串
// 确保内存访问不会出问题
// 只有一种构造方式，从上面的类似 ROS 的结构体初始化
HeaderInShm::HeaderInShm() { memset(s_frame_id, 0, 64); }

HeaderInShm::HeaderInShm(const Header &header)
    : u_seq_num(header.u_seq_num)
{
  stamp = header.stamp;
  strcpy(s_frame_id, header.frame_id.c_str());
}

void HeaderInShm::clear()
{
  u_seq_num = 0;
  stamp = BgsTime();
  memset(s_frame_id, 0, 64);
}

PointField::PointField()
    : name()
    , offset(0)
    , datatype(INT8)
    , count(0)
{
}

PointField::~PointField() {}

PointField::PointField(const PointFieldInShm &point_field_in_shm)
{
  init_with_point_field_in_shm(point_field_in_shm);
}

PointField &PointField::operator=(const PointFieldInShm &point_field_in_shm)
{
  init_with_point_field_in_shm(point_field_in_shm);
  return *this;
}

void PointField::init_with_point_field_in_shm(
  const PointFieldInShm &point_field_in_shm)
{
  name = std::string(point_field_in_shm.name);
  offset = point_field_in_shm.offset;
  datatype = point_field_in_shm.datatype;
  count = point_field_in_shm.count;
}

void PointCloud2MsgInShm::clear()
{
  header.clear();
  height = width = 0;

  data_count = 0;
  memset(data, 0, sizeof(data));
}

void PointCloud2MsgInShm::init_with_point_cloud_msg(
  const PointCloud2Msg &point_cloud_2_msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(point_cloud_2_msg, pcl_pc2);

  header = HeaderFromPclToBgs(pcl_pc2.header);
  height = pcl_pc2.height;
  width = pcl_pc2.width;
  is_bigendian = pcl_pc2.is_bigendian;
  point_step = pcl_pc2.point_step;
  row_step = pcl_pc2.row_step;
  is_dense = pcl_pc2.is_dense;
  fields_count = pcl_pc2.fields.size();
  data_count = pcl_pc2.data.size();

  for (uint32_t i = 0; i < fields_count; ++i)
    fields[i] = FieldFromPclToBgs(pcl_pc2.fields.at(i));
  if (!pcl_pc2.data.empty())
    memcpy(data, pcl_pc2.data.data(),
           data_count
             * sizeof(uint8_t)); // data() 返回这个vector里面的数据头指针
}

PointCloud2MsgInShm &PointCloud2MsgInShm::
operator=(const PointCloud2Msg &point_cloud_2_msg)
{
  init_with_point_cloud_msg(point_cloud_2_msg);
  return *this;
}

PointCloud2MsgInShm::PointCloud2MsgInShm(
  const PointCloud2Msg &point_cloud_2_msg)
{
  init_with_point_cloud_msg(point_cloud_2_msg);
}

void PointCloud2MsgInShm::to_point_cloud_msg(PointCloud2Msg &msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_pc2.header = HeaderFromBgsShmToPcl(header);

  pcl_pc2.height = height;
  pcl_pc2.width = width;
  pcl_pc2.is_bigendian = is_bigendian;
  pcl_pc2.point_step = point_step;
  pcl_pc2.row_step = row_step;
  pcl_pc2.is_dense = is_dense;

  pcl_pc2.fields.resize(fields_count);
  for (uint32_t i = 0; i < fields_count; ++i) {
    Bgs::PointField f = fields[i];
    pcl_pc2.fields[i] = FieldFromBgsToPcl(f);
  }
  pcl_pc2.data.resize(data_count);
  memcpy(pcl_pc2.data.data(), data, data_count * sizeof(uint8_t));

  if (!pcl_pc2.fields.empty()) {
    pcl::fromPCLPointCloud2(pcl_pc2, msg);
  } else {
    msg.clear();
  }
}

// PointCloud2Msg& PointCloud2Msg::operator=( const PointCloud2MsgInShm&
// point_cloud_in_shm )
// {
// 	header = point_cloud_in_shm.header;
// 	height = point_cloud_in_shm.height;
// 	width = point_cloud_in_shm.width;
// 	is_bigendian = point_cloud_in_shm.is_bigendian;
// 	point_step = point_cloud_in_shm.point_step;
// 	row_step = point_cloud_in_shm.row_step;
// 	is_dense = point_cloud_in_shm.is_dense;
//
// 	fields.resize( point_cloud_in_shm.fields_count );
// 	for( uint32_t i = 0; i < point_cloud_in_shm.fields_count; ++i )
// 		fields.at(i) = point_cloud_in_shm.fields[i];
//
// 	// 这样的操作可能效率会高一些，不用循环赋值
// 	data.resize( point_cloud_in_shm.data_count );
// // 	for( uint32_t i = 0; i < point_cloud_in_shm.data_count; ++i )
// // 		data.at(i) = point_cloud_in_shm.data[i];
// 	memcpy( data.data(), point_cloud_in_shm.data, point_cloud_in_shm.data_count
// * sizeof(uint8_t) );
//
// 	return *this;
// }

HeaderInShm &HeaderInShm::operator=(const Header &header)
{
  u_seq_num = header.u_seq_num;
  stamp = header.stamp;
  memset(s_frame_id, 0, 64);
  strcpy(s_frame_id, header.frame_id.c_str());

  return *this;
}

LaserScanMsg &LaserScanMsg::operator=(const LaserScanInShm &laser_scan_in_shm)
{
  header = laser_scan_in_shm.header;

  angle_min = laser_scan_in_shm.angle_min;
  angle_max = laser_scan_in_shm.angle_max;
  angle_increment = laser_scan_in_shm.angle_increment;
  time_increment = laser_scan_in_shm.time_increment;
  scan_time = laser_scan_in_shm.scan_time;

  range_max = laser_scan_in_shm.range_max;
  range_min = laser_scan_in_shm.range_min;

  laser_scan_count = laser_scan_in_shm.laser_scan_count;

  // 	#pragma omp parallel for
  for (int i = 0; i < SCAN_SIZE; ++i) {
    ranges[i] = laser_scan_in_shm.ranges[i];
    intensities[i] = laser_scan_in_shm.intensities[i];
  }

  // 	memcpy( ranges, laser_scan_in_shm.ranges, sizeof(ranges) );
  // 	memcpy( intensities, laser_scan_in_shm.intensities, sizeof(intensities) );

  return *this;
}

void LaserScanInShm::clear()
{
  header.clear();
  angle_min = angle_max = angle_increment = 0.;
  time_increment = scan_time = 0.;
  range_max = range_min = 0.;
  laser_scan_count = 0;

  memset(ranges, 0, sizeof(ranges));
  memset(intensities, 0, sizeof(intensities));
}

void LaserScanInShm::init_with_laser_scan_msg(const LaserScanMsg &laser_scan)
{
  header = laser_scan.header;

  // printf("LASER SCAN operator= : max = %f, min = %f \n",
  // laser_scan.angle_max, laser_scan.angle_min);

  angle_min = laser_scan.angle_min;
  angle_max = laser_scan.angle_max;
  angle_increment = laser_scan.angle_increment;

  time_increment = laser_scan.time_increment;
  scan_time = laser_scan.scan_time;

  range_min = laser_scan.range_min;
  range_max = laser_scan.range_max;

  laser_scan_count = laser_scan.laser_scan_count;

#if 1
  //      #pragma omp parallel for
  // 这样赋值效率会偏低但是比较保险，不会出现在对共享内存访问时出现地址错误的问题
  for (int i = 0; i < SCAN_SIZE; ++i) {
    ranges[i] = laser_scan.ranges[i];
    intensities[i] = laser_scan.intensities[i];
  }
#else
  memcpy(ranges, laser_scan.ranges, sizeof(ranges));
  memcpy(intensities, laser_scan.intensities, sizeof(intensities));
#endif
}

OdometryMsg &OdometryMsg::operator=(const OdomMsgInShm &odom_msg_in_shm)
{
  header = odom_msg_in_shm.header;
  child_frame_id = odom_msg_in_shm.child_frame_id;

  memcpy(&pose, &(odom_msg_in_shm.pose), sizeof(pose));
  memcpy(&twist, &(odom_msg_in_shm.twist), sizeof(twist));

  return *this;
}

void OdomMsgInShm::init_with_odom_msg(const OdometryMsg &odom_msg)
{
  header = odom_msg.header;
  strcpy(child_frame_id, odom_msg.child_frame_id.c_str());

  memcpy(&pose, &(odom_msg.pose), sizeof(pose));
  memcpy(&twist, &(odom_msg.twist), sizeof(twist));
}

ImuMsg &ImuMsg::operator=(const ImuMsgInShm &imu_msg_in_shm)
{
  header = imu_msg_in_shm.header;

  orientation = imu_msg_in_shm.orientation;
  angular_velocity = imu_msg_in_shm.angular_velocity;
  linear_acceleration = imu_msg_in_shm.linear_acceleration;

  roll = imu_msg_in_shm.roll;
  pitch = imu_msg_in_shm.pitch;
  yaw = imu_msg_in_shm.yaw;

  for (int i = 0; i < 9; ++i) {
    orientation_covariance[i] = imu_msg_in_shm.orientation_covariance[i];
    angular_velocity_covariance[i] =
      imu_msg_in_shm.angular_velocity_covariance[i];
    linear_acceleration_covariance[i] =
      imu_msg_in_shm.linear_acceleration_covariance[i];
  }

  return *this;
}

void ImuMsgInShm::init_with_imu_msg(const ImuMsg &imu_msg)
{
  header = imu_msg.header;

  roll = imu_msg.roll;
  pitch = imu_msg.pitch;
  yaw = imu_msg.yaw;
  memcpy(&orientation, &(imu_msg.orientation), sizeof(Quaternion));
  memcpy(&orientation_covariance, &(imu_msg.orientation_covariance),
         sizeof(Convariance));

  memcpy(&angular_velocity, &(imu_msg.angular_velocity), sizeof(Vector3));
  memcpy(&angular_velocity_covariance, &(imu_msg.angular_velocity_covariance),
         sizeof(Convariance));

  memcpy(&linear_acceleration, &(imu_msg.linear_acceleration), sizeof(Vector3));
  memcpy(&linear_acceleration_covariance,
         &(imu_msg.linear_acceleration_covariance), sizeof(Convariance));
}

MotorMsg &MotorMsg::operator=(const MotorMsgInShm &motor_msg_in_shm)
{
  header = motor_msg_in_shm.header;

  motor_cL = motor_msg_in_shm.motor_cL;
  motor_cR = motor_msg_in_shm.motor_cR;

  motor_pL = motor_msg_in_shm.motor_pL;
  motor_pR = motor_msg_in_shm.motor_pR;

  motor_speedL = motor_msg_in_shm.motor_speedL;
  motor_speedR = motor_msg_in_shm.motor_speedR;

  is_new_command = motor_msg_in_shm.is_new_command;
  d_wait_secs = motor_msg_in_shm.d_wait_secs;

  return *this;
}

void MotorMsgInShm::clear()
{
  header.clear();

  motor_cL = motor_cR = 0;
  motor_pL = motor_pR = 0;
  motor_speedL = motor_speedR = 0;

  is_new_command = FALSE;
}

void MotorMsgInShm::init_with_motor_msg(const MotorMsg &motor_msg)
{
  header = motor_msg.header;

  motor_pL = motor_msg.motor_pL;
  motor_pR = motor_msg.motor_pR;
  motor_cL = motor_msg.motor_cL;
  motor_cR = motor_msg.motor_cR;
  motor_speedL = motor_msg.motor_speedL;
  motor_speedR = motor_msg.motor_speedR;

  is_new_command = motor_msg.is_new_command;
  d_wait_secs = motor_msg.d_wait_secs;
}

GpsMsg &GpsMsg::operator=(const GpsMsgInShm &gps_msg_in_shm)
{
  header = gps_msg_in_shm.header;

  gps_position = gps_msg_in_shm.gps_position;
  gps_gmode = gps_msg_in_shm.gps_gmode;
  gps_pmode = gps_msg_in_shm.gps_pmode;

  has_new_pos = gps_msg_in_shm.has_new_pos;

  gps_heading = gps_msg_in_shm.gps_heading;
  gps_type = gps_msg_in_shm.gps_type;
  status = gps_msg_in_shm.status;

  return *this;
}

void GpsMsgInShm::init_with_gps_msg(const GpsMsg &gps_msg)
{
  header = gps_msg.header;
  memcpy(&gps_position, &(gps_msg.gps_position), sizeof(gps_position));

  gps_heading = gps_msg.gps_heading;
  gps_pmode = gps_msg.gps_pmode;
  gps_gmode = gps_msg.gps_gmode;
  gps_type = gps_msg.gps_type;
  has_new_pos = gps_msg.has_new_pos;
  status = gps_msg.status;
}

#define SEM_OP_ERROR -1
#define SEM_OP_SUCCEED 0
#define SEM_OP_TIME_OUT 1
int semaphore_p(int32_t sem_id)
{
  //对信号量做减1操作，即等待P（sv）
  struct sembuf sem_b = {.sem_num = 0, .sem_op = -1, .sem_flg = 0 };

  if (semop(sem_id, &sem_b, 1) == -1) {
    // 这里有一种特殊情况，就是在 semop 等待信号量时
    // 进程捕获到某个信号（如crtl+c的SIGINT信号）后可能会使 semop
    // 函数立即返回一个错误 —— SINTR 这时为了正常运行原有的功能，重启 semop
    // 可能可以避免该错误
    if (errno == EINTR) {
      // 再次尝试 semop
      // 			PRINT_WARNING("try again!");
      if (semop(sem_id, &sem_b, 1) == -1) {
        perror("semaphore_p failed");
        return SEM_OP_ERROR;
      } else
        return SEM_OP_SUCCEED;
    } else {
      perror("semaphore_p failed");
      return SEM_OP_ERROR;
    }
  }

  return SEM_OP_SUCCEED;
}

int semaphore_p_timed(int32_t sem_id, struct timespec *timeout)
{
  if (timeout == NULL)
    return semaphore_p(sem_id);

  struct sembuf sem_b = {.sem_num = 0, .sem_op = -1, .sem_flg = 0 };

  int ret = SEM_OP_SUCCEED;
  if (semtimedop(sem_id, &sem_b, 1, timeout) == -1) {
    // 这里有一种特殊情况，就是在 semop 等待信号量时
    // 进程捕获到某个信号（如crtl+c的SIGINT信号）后可能会使 semop
    // 函数立即返回一个错误 —— SINTR 这时为了正常运行原有的功能，重启 semop
    // 可能可以避免该错误
    switch (errno) {
    case EINTR:

      if (semtimedop(sem_id, &sem_b, 1, timeout) == -1) {
        if (errno == EAGAIN) {
          ret = SEM_OP_TIME_OUT;
          break;
        }
        perror("semaphore_p_timed failed");
        ret = SEM_OP_ERROR;
      } else
        ret = SEM_OP_SUCCEED;

      break;

    case EAGAIN:
      // 				PRINT_DEBUG(" time out ");
      ret = SEM_OP_TIME_OUT;
      break;

    default:
      perror("semaphore_p_timed failed");
      ret = SEM_OP_ERROR;
      break;
    }
  }

  return ret;
}

int semaphore_v(int32_t sem_id)
{
  //这是一个释放操作，它使信号量变为可用，即发送信号V（sv）
  struct sembuf sem_b = {.sem_num = 0, .sem_op = 1, .sem_flg = 0 };

  if (semop(sem_id, &sem_b, 1) == -1) {
    perror("semaphore_v failed");
    // fprintf(stderr, "semaphore_v failed\n");
    return SEM_OP_ERROR;
  }
  return SEM_OP_SUCCEED;
}

void del_semvalue(int32_t sem_id)
{
  //删除信号量
  union semun sem_union;

  if (semctl(sem_id, 0, IPC_RMID, sem_union) == -1)
    fprintf(stderr, "Failed to delete semaphore\n");
}

int32_t init_sem(int32_t sem_id, int32_t sem_value)
{
  union semun sem_union;
  sem_union.val = sem_value;
  if (semctl(sem_id, 0, SETVAL, sem_union) == -1) {
    return SEM_OP_ERROR;
  }

  return SEM_OP_SUCCEED;
}

int32_t get_sem_value(int32_t sem_id, int32_t *sem_value)
{
  union semun sem_union;
  int ret = semctl(sem_id, 0, GETVAL, sem_union);

  if (ret == -1) {
    return SEM_OP_ERROR;
  }

  *sem_value = ret;
  return SEM_OP_SUCCEED;
}

#undef SEM_OP_ERROR
#undef SEM_OP_SUCCEED

} // namespace Bgs
// namespace Bgs
