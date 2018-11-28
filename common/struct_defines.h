#pragma once

#include <boost/concept_check.hpp>
#include <boost/shared_ptr.hpp>

#include <pcl/PCLHeader.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <sys/sem.h>
#include <sys/types.h>
#include <vector>

#include "bgs_time.h"

// using namespace std;
/*!
 * @namespace Bgs
 * @brief namespace for many structs and classes
 *
 * @todo should be replaced by a set of more
 * specific namespaces like sensor_msgs ...
 */
namespace Bgs
{

struct HeaderInShm;
struct Header {
public:
  Header();
  ~Header() = default;

  Header &operator=(const HeaderInShm &header_in_shm);
  Header &operator=(const Header &header);

  uint32_t u_seq_num;
  BgsTime stamp;
  std::string frame_id;
};

// 在共享内存中，建议使用char数组而不是string来保存字符串
// 确保内存访问不会出问题
// 只有一种构造方式，从上面的类似 ROS 的结构体初始化
struct HeaderInShm {
public:
  HeaderInShm();
  HeaderInShm(const Header &header);
  HeaderInShm &operator=(const Header &header);
  ~HeaderInShm() = default;

  uint32_t u_seq_num;
  BgsTime stamp;
  char s_frame_id[64];

  void clear();
};

typedef struct {
  uint32_t secs;
  uint32_t nsecs;
} TimeStamp;

struct MotorMsgInShm;
struct MotorMsg {
public:
  MotorMsg() = default;
  MotorMsg &operator=(const MotorMsgInShm &motor_msg_in_shm);

  Header header;

  // motor past encoder
  int16_t motor_pL;
  int16_t motor_pR;

  // motor current encoder
  int16_t motor_cL;
  int16_t motor_cR;

  // motor target speed
  int16_t motor_speedL;
  int16_t motor_speedR;

  int32_t is_new_command;
  double d_wait_secs;
  double reserved[9];
};
typedef boost::shared_ptr<MotorMsg> MotorMsgPtr;
typedef boost::shared_ptr<MotorMsg const> MotorMsgConstPtr;

struct MotorMsgInShm {
  MotorMsgInShm() = default;
  MotorMsgInShm(const MotorMsg &motor_msg) { init_with_motor_msg(motor_msg); }
  ~MotorMsgInShm() = default;

  MotorMsgInShm &operator=(const MotorMsg &motor_msg)
  {
    init_with_motor_msg(motor_msg);
    return *this;
  }

  HeaderInShm header;

  // motor past encoder
  int16_t motor_pL;
  int16_t motor_pR;

  // motor current encoder
  int16_t motor_cL;
  int16_t motor_cR;

  // motor target speed
  int16_t motor_speedL;
  int16_t motor_speedR;

  int32_t is_new_command;
  double d_wait_secs;
  double reserved[9];

  void clear();

private:
  void init_with_motor_msg(const MotorMsg &motor_msg);
};

struct LaserScanInShm;
struct LaserScanMsg {
  LaserScanMsg &operator=(const LaserScanInShm &laser_scan_in_shm);

  Header header;

  float angle_min;
  float angle_max;
  float angle_increment;

  float time_increment;
  float scan_time;

  float range_min;
  float range_max;
  float reserved[5];

  int32_t laser_scan_count;
  int32_t reserved2[9];

  float ranges[SCAN_SIZE];
  float intensities[SCAN_SIZE];
};
typedef std::shared_ptr<LaserScanMsg> LaserScanPtr;
typedef std::shared_ptr<LaserScanMsg const> LaserScanConstPtr;

struct LaserScanInShm {
  LaserScanInShm() = default;
  LaserScanInShm(const LaserScanMsg &laser_scan)
  {
    init_with_laser_scan_msg(laser_scan);
  }
  ~LaserScanInShm() = default;

  LaserScanInShm &operator=(const LaserScanMsg &laser_scan)
  {
    init_with_laser_scan_msg(laser_scan);
    return *this;
  }

  HeaderInShm header;

  float angle_min;
  float angle_max;
  float angle_increment;

  float time_increment;
  float scan_time;

  float range_min;
  float range_max;
  float reserved[5];

  int32_t laser_scan_count;
  int32_t reserved2[9];

  float ranges[SCAN_SIZE];
  float intensities[SCAN_SIZE];

  void clear();

private:
  void init_with_laser_scan_msg(const LaserScanMsg &laser_scan);
};

enum DataType {
  INT8 = 1,
  UINT8 = 2,
  INT16 = 3,
  UINT16 = 4,
  INT32 = 5,
  UINT32 = 6,
  FLOAT32 = 7,
  FLOAT64 = 8
};

struct PointFieldInShm;
struct PointField {
  std::string name;
  uint32_t offset;
  uint8_t datatype;
  uint32_t count;

  PointField();
  PointField(const PointFieldInShm &point_field_in_shm);
  ~PointField();

  PointField &operator=(const PointFieldInShm &point_field_in_shm);

private:
  void init_with_point_field_in_shm(const PointFieldInShm &point_field_in_shm);
};

struct PointFieldInShm {
  char name[128];
  uint32_t offset;
  uint8_t datatype;
  uint32_t count;

  PointFieldInShm(){};
  PointFieldInShm(const PointField &point_field)
  {
    init_with_point_field(point_field);
  }

  PointFieldInShm &operator=(const PointField &point_field)
  {
    init_with_point_field(point_field);
    return *this;
  }

  ~PointFieldInShm(){};

private:
  void init_with_point_field(const PointField &point_field)
  {
    strcpy(name, point_field.name.c_str());
    offset = point_field.offset;
    datatype = point_field.datatype;
    count = point_field.count;
  }
};

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud2Msg;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud2Ptr;
typedef pcl::PointCloud<pcl::PointXYZI>::ConstPtr PointCloud2ConstPtr;

#define ROBOSENSE_16_DATA_COUNT_ 1032192
#define ROBOSENSE_16_FIELD_COUNT_ 4
struct PointCloud2MsgInShm {
  HeaderInShm header;
  uint32_t height;
  uint32_t width;

  uint32_t fields_count;
  PointFieldInShm fields[ROBOSENSE_16_FIELD_COUNT_];
  bool is_bigendian;
  uint32_t point_step;
  uint32_t row_step;
  uint32_t data_count;
  uint8_t data[ROBOSENSE_16_DATA_COUNT_];
  bool is_dense;

  PointCloud2MsgInShm()
      : height(0)
      , width(0)
  {
  }

  PointCloud2MsgInShm(const PointCloud2Msg &point_cloud_2_msg);
  PointCloud2MsgInShm &operator=(const PointCloud2Msg &point_cloud_2_msg);
  void to_point_cloud_msg(PointCloud2Msg &msg);
  ~PointCloud2MsgInShm() {}

  void clear();

private:
  void init_with_point_cloud_msg(const PointCloud2Msg &point_cloud_2_msg);
};

// 四元数
typedef struct {
  double x;
  double y;
  double z;
  double w;
} Quaternion;

struct Point2d {
public:
  double x, y;

  Point2d()
      : x(0.)
      , y(0.)
  {
  }
  Point2d(double x_, double y_)
      : x(x_)
      , y(y_)
  {
  }

  void set_xy(double d_x, double d_y)
  {
    x = d_x;
    y = d_y;
  }

  inline double distance_to_point2d(const Point2d &point_) const
  {
    return sqrt(pow(point_.x - x, 2.) + pow(point_.y - y, 2.));
  }

  Point2d mid_point_with(const Point2d &vector) const
  {
    Point2d point;
    point.set_xy((vector.x + x) * 0.5, (vector.y + y) * 0.5);
    return point;
  }

  Point2d operator-(const Point2d &b) { return Point2d(x - b.x, y - b.y); }
  Point2d operator+(const Point2d &b) { return Point2d(x + b.x, y + b.y); }

  bool operator==(const Point2d &b)
  {
    return (DOUBLE_EQUAL(x, b.x) && DOUBLE_EQUAL(y, b.y));
  }

  std::string DebugString() const
  {
    std::ostringstream out;
    out << "[ " << x << ", " << y << " ]";
    return out.str();
  }
};
typedef Point2d Vector2d;

struct Pose2d {
  Point2d position;
  double angle_yaw; // between -pi/2 ~ pi/2
};
typedef Pose2d Coordinate;

struct Vector3 {
public:
  Vector3() { x = y = z = 0.; }

  Vector3(double x_, double y_, double z_)
  {
    x = x_;
    y = y_;
    z = z_;
  }

  double x, y, z;

  inline double distance_to_vector3(const Vector3 &vector)
  {
    return sqrt(pow(vector.x - x, 2.) + pow(vector.y - y, 2.)
                + pow(vector.z - z, 2.));
  }

  inline double distance_to_point3d(const Vector3 &vector)
  {
    return distance_to_vector3(vector);
  }

  void set_xy(double d_x, double d_y)
  {
    x = d_x;
    y = d_y;
  }

  void set_xyz(double d_x, double d_y, double d_z)
  {
    x = d_x;
    y = d_y;
    z = d_z;
  }

  Vector3 mid_point_with(const Vector3 &vector)
  {
    Vector3 point;
    point.set_xyz((vector.x + x) * 0.5, (vector.y + y) * 0.5,
                  (vector.z + z) * 0.5);
    return point;
  }

  Vector3 operator+(const Vector3 &b)
  {
    return Vector3(x + b.x, y + b.y, z + b.z);
  }

  Vector3 operator-(const Vector3 &b)
  {
    return Vector3(x - b.x, y - b.y, z - b.z);
  }

  std::string DebugString() const
  {
    std::ostringstream out;
    out << x << "," << y << "," << z;
    return out.str();
  }
};
typedef Vector3 Point;
typedef Vector3 Point3d;

typedef struct {
  int16_t left;
  int16_t right;
} MotorSpeed;

// 机器人位姿信息，包括位置和姿态
typedef struct {
  Point position;
  Quaternion orientation;
} Pose;

typedef struct {
  int32_t trajectory_id;
  int32_t submap_index;
  int32_t submap_version;
  Bgs::Pose pose;
} SubmapEntry;

typedef struct {
  Header header;
  std::vector<SubmapEntry> submap;
} SubmapList;
typedef std::shared_ptr<SubmapList> SubmapListPtr;
typedef std::shared_ptr<SubmapList const> SubmapListConstPtr;

typedef struct {
  std::vector<uint8_t> cells;
  int32_t width;
  int32_t height;

  double resolution;
  Bgs::Pose slice_pose;
} SubmapTexture;

struct SubmapQuery {
  class Request
  {
  public:
    int32_t trajectory_id;
    int32_t submap_index;
  };

  class Response
  {
  public:
    int32_t submap_version;
    std::vector<SubmapTexture> textures;
    std::string error_message;
  };

  Request request;
  Response response;
};

struct SubmapCloudQuery {
  class Request
  {
  public:
    int32_t trajectory_id;
    int32_t submap_index;
    float min_probability;
    bool high_resolution;
  };

  class Response
  {
  public:
    int32_t submap_version;
    float resolution;
    bool finished;
    Bgs::PointCloud2Msg cloud;
  };

  Request request;
  Response response;
};

struct MapMetaData {
  BgsTime map_load_time;
  float resolution;
  uint32_t width;
  uint32_t height;
  Pose origin;
};

struct OccupancyGrid {
  Header header;

  MapMetaData info;
  std::vector<int8_t> data;
};

#define GPS_STATUS_NO_FIX -1
#define GPS_STATUS_FIX 0
#define GPS_STATUS_SBAS_FIX 1
#define GPS_STATUS_GBAS_FIX 2

struct GpsMsgInShm;
struct GpsMsg {
public:
  GpsMsg &operator=(const GpsMsgInShm &gps_msg_in_shm);

  Header header;

  // gps position
  Vector3 gps_position;

  // gps heading
  double gps_heading;

  // gps mode
  uint32_t gps_pmode;
  uint32_t gps_gmode;

  // double gps activated
  uint32_t gps_type;

  int16_t has_new_pos;
  int16_t status;
  int16_t reserved2[2];
  double reserved[9];
};
typedef boost::shared_ptr<GpsMsg> GpsMsgPtr;
typedef boost::shared_ptr<GpsMsg const> GpsMsgConstPtr;

struct GpsMsgInShm {
  GpsMsgInShm() = default;
  GpsMsgInShm(const GpsMsg &gps_msg) { init_with_gps_msg(gps_msg); }
  ~GpsMsgInShm() = default;

  GpsMsgInShm &operator=(const GpsMsg &gps_msg)
  {
    init_with_gps_msg(gps_msg);
    return *this;
  }

  HeaderInShm header;

  // gps position
  Vector3 gps_position;

  // gps heading
  double gps_heading;

  // gps mode
  uint32_t gps_pmode;
  uint32_t gps_gmode;

  // double gps activated
  uint32_t gps_type;

  int16_t has_new_pos;
  int16_t status;
  int16_t reserved2[2];
  double reserved[9];

private:
  void init_with_gps_msg(const GpsMsg &gps_msg);
};

// 三维协方差矩阵
typedef double Convariance[9];
struct ImuMsgInShm;
struct ImuMsg {
public:
  ImuMsg &operator=(const ImuMsgInShm &imu_msg_in_shm);

  Header header;

  // gesture
  Quaternion orientation;
  Convariance orientation_covariance;

  // angular_velocity
  // rad/s
  Vector3 angular_velocity;
  Convariance angular_velocity_covariance;

  // linear_acceleration
  // m/s^2
  Vector3 linear_acceleration;
  Convariance linear_acceleration_covariance;

  double roll;
  double pitch;
  double yaw;

  double reserved[7];
};
typedef boost::shared_ptr<ImuMsg> ImuMsgPtr;
typedef boost::shared_ptr<ImuMsg const> ImuMsgConstPtr;

struct ImuMsgInShm {
  ImuMsgInShm() = default;
  ImuMsgInShm(const ImuMsg &imu_msg) { init_with_imu_msg(imu_msg); }
  ~ImuMsgInShm() = default;

  ImuMsgInShm &operator=(const ImuMsg &imu_msg)
  {
    init_with_imu_msg(imu_msg);
    return *this;
  }

  HeaderInShm header;
  // TimeStamp	time_stamp;	// 64bits

  // gesture
  Quaternion orientation;
  Convariance orientation_covariance;

  // angular_velocity
  // rad/s
  Vector3 angular_velocity;
  Convariance angular_velocity_covariance;

  // linear_acceleration
  // m/s^2
  Vector3 linear_acceleration;
  Convariance linear_acceleration_covariance;

  double roll;
  double pitch;
  double yaw;

  double reserved[7];

private:
  void init_with_imu_msg(const ImuMsg &imu_msg);
};

typedef struct {
  Vector3 linear;
  Vector3 angular;
} Twist;

struct OdomMsgInShm;
struct OdometryMsg {
  OdometryMsg &operator=(const OdomMsgInShm &odom_msg_in_shm);

  Header header;
  std::string child_frame_id;

  struct {
    Pose pose;
    double convariance[36];
  } pose;

  struct {
    Twist twist;
    double convariance[36];
  } twist;

private:
};
typedef boost::shared_ptr<OdometryMsg> OdometryPtr;
typedef boost::shared_ptr<OdometryMsg const> OdometryConstPtr;

struct OdomMsgInShm {
  OdomMsgInShm() = default;
  OdomMsgInShm(const OdometryMsg &odom_msg) { init_with_odom_msg(odom_msg); }
  ~OdomMsgInShm() = default;

  OdomMsgInShm &operator=(const OdometryMsg &odom_msg)
  {
    init_with_odom_msg(odom_msg);
    return *this;
  }

  HeaderInShm header;
  char child_frame_id[64];

  struct {
    Pose pose;
    double convariance[36];
  } pose;

  struct {
    Twist twist;
    double convariance[36];
  } twist;

private:
  void init_with_odom_msg(const OdometryMsg &odom_msg);
};

typedef struct {
  std::string laser_scan_topic;
  std::string multi_echo_laser_scan_topic;
  std::string point_cloud2_topic;
  std::string imu_topic;
  std::string odometry_topic;
  std::string gps_topic;
  std::string motor_topoc;
} SensorTopics;

typedef struct {
  Vector3 translation;
  Quaternion rotation;
} BgsTransform;

typedef struct {
  Header header;
  std::string child_frame_id;
  BgsTransform transform;
} TransformStamped;

struct AllSensorMsgs {
  BgsTime system_stamp;

  LaserScanConstPtr shared_laser_scan;
  ImuMsgConstPtr shared_imu;
  OdometryConstPtr shared_odom;
  GpsMsgConstPtr shared_gps;
  MotorMsgConstPtr shared_motor;
  PointCloud2ConstPtr shared_point_cloud;
};

struct GlobalShm {
  volatile int32_t n_carto_pid;
  volatile int32_t carto_sem_id;
  volatile int32_t shm_sem_id;
  volatile int32_t imu_sem_id;
  volatile int32_t lidar_sem_id;

  struct {
    int8_t all_ready;
    int8_t providing_location;
    int8_t current_pose_property;
    int8_t got_rough_direction;
    int32_t remaining_work_count;
  } carto_status;

  struct {
    int8_t all_ready;
    int8_t lidar_thread_running;
    int8_t reservedm[6];
  } driver_status;

  struct {
    int8_t all_ready;
    int8_t reserved[7];
  } controller_status;

  union {
    struct {
      uint8_t lidar_error;
      uint8_t imu_error;
      uint8_t reserved[6];
    } error;
    uint64_t all_error;
  };

  volatile int32_t gps_sem_id;

  BgsTime stamp;

  // sensor data
  LaserScanInShm scan_msg;
  ImuMsgInShm imu_msg;
  OdomMsgInShm odom_msg;
  GpsMsgInShm gps_msg;
  MotorMsgInShm motor_msg;
  PointCloud2MsgInShm point_cloud2_msg;

  // results
  Pose current_pose;

  struct {
    struct {
      double x, y;
    } translation;

    struct {
      double mat00, mat01;
      double mat10, mat11;
    } rotation_matrix;
  } corrd_transfrom_carto_to_gps;

  float velocity;
  float angle_velocity;

  double reserved2[3];
};

union semun {
  int val;              // cmd == SETVAL
  struct semid_ds *buf; // cmd == IPC_SET或者 cmd == IPC_STAT
  ushort *array;        // cmd == SETALL，或 cmd = GETALL
  struct seminfo *buf_;
};

int semaphore_p(int32_t sem_id);
int semaphore_v(int32_t sem_id);
int semaphore_p_timed(int32_t sem_id, struct timespec *timeout);
void del_semvalue(int32_t sem_id);
int32_t init_sem(int32_t sem_id, int32_t sem_value);
int32_t get_sem_value(int32_t sem_id, int32_t *sem_value);

#define LOCK_SHM Bgs::semaphore_p(g_ptr_shm->shm_sem_id)
#define UNLOCK_SHM Bgs::semaphore_v(g_ptr_shm->shm_sem_id)

inline void msleep(int32_t ms) { usleep(ms * 1000); }

} // namespace Bgs
