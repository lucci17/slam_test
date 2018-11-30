/// @file LidarCollector.h
/// @brief Head file for LidarCollectors.
///
/// @author edward_liu
/// @date 2018-10-06

#pragma once

#include "LMS1xx/LMS1xx.h"
#include "rslidar/convert.h"
#include "rslidar/rsdriver.h"
#include "struct_defines.h"

#include <functional>
#include <thread>

/// @brief namespace for all hardware
/// in this project
///
/// including many collectors, such as
/// - lidar collector
/// - imu collector
/// - gps collector
/// - motor (collector)
///
/// and some hardware base class such as
/// - gnss colution
/// - serial
///
/// and data bag fuctions for simulation
namespace hardware
{
#define _2D_LIDAR_ 0
#define _3D_LIDAR_ 1

using LidarCallback2d = std::function<int32_t(void)>;
using LidarCallback3d = std::function<int32_t(void)>;

/// \brief class LidarCollector
///
/// virtual class for all 2d and 3d lidar
/// get point cloud2 for 3d lidar
/// get scan msg for 2d lidar
class LidarCollector
{
public:
  LidarCollector(){};
  ~LidarCollector(){};

  /// \brief initilisation
  ///
  /// \param host ip address of the lidar, example: 192.168.0.1
  /// \param port udp port
  /// \return BGS_OK -- succeed in initilisation;
  /// \return BGS_ERROR -- failed
  virtual int32_t init(char host[], int32_t port) = 0;

  /// \brief reset the hardware in a right way
  ///
  /// \note you should use this function
  /// before the object destroyed
  virtual void deInit() = 0;

  /// \brief get Bgs::LaserScanMsg data
  ///
  /// \return
  /// - BGS_ERROR failed when getting the laser scan message
  /// - BGS_OK    succeed
  ///
  /// \note it is for a 2d lidar
  virtual int32_t get_scan_msg(Bgs::LaserScanMsg *scan_msg) = 0;

  /// \brief get laser config
  ///
  /// \return
  /// - BGS_ERROR failed when getting the laser scan config
  /// - BGS_OK    succeed
  ///
  /// \note it is for a 2d lidar
  virtual int32_t get_scan_cfg(Bgs::LaserScanMsg *scan_msg) = 0;

  /// \brief get point cloud2 config into
  /// a cloud2 shared pointer
  ///
  /// \return
  /// - BGS_ERROR failed when getting the pointcloud2 config
  /// - BGS_OK    succeed
  ///
  /// \note it is for a 3d lidar
  virtual int32_t get_point_cloud2_cfg(Bgs::PointCloud2Ptr &cloud) = 0;

  /// \brief get point cloud2 data into
  /// a cloud2 shared pointer
  ///
  /// \return
  /// - BGS_ERROR failed when getting the pointcloud2 data
  /// - BGS_OK    succeed
  ///
  /// \note it is for a 3d lidar
  virtual int32_t get_point_cloud2(Bgs::PointCloud2Ptr &cloud) = 0;

  /// \brief lidar type
  /// - 0 : 2d
  /// - 1 : 3d
  ///
  /// \warning you should not modify it by your own
  int32_t type_;

  /// \brief it is a callback function
  /// called when the error happens
  /// such as losing connection
  ///
  /// \note it can be empty
  std::function<void(void)> on_error_;
};

/// \brief class LMS1Collector
///
/// it a class for 2d lidar
/// sick lms1xx
class LMS1Collector : public LidarCollector
{
private:
  LASER::LMS1xx laser;
  LASER::scanData data;
  LASER::scanOutputRange outputRange;
  LASER::scanCfg cfg;

  /*!
   * @brief inner thread
   *
   * inner thread for updating data
   * since the lidar communication is in udp
   * we should always wait the lidar for data
   */
  std::thread thread_update_data_;

  /*!
   * @brief callback function
   *
   * when the data comming, this callback
   * will be called
   */
  LidarCallback2d callback_;

  bool exit_thread_;
  bool init_finished_;

  /*!
   * @brief shared_ptr of the laser scan msg
   *
   * used in the callback function
   * when get scan data,
   * then we use the pointer to send the data out to use
   */
  Bgs::LaserScanPtr laser_scan_;

public:
  LMS1Collector(Bgs::LaserScanPtr &laser_scan_ptr,
                const LidarCallback2d &callback)
      : thread_update_data_(std::bind(&LMS1Collector::update_data, this))
      , callback_(callback)
      , exit_thread_(false)
      , init_finished_(false)
      , laser_scan_(laser_scan_ptr)
  {
    type_ = _2D_LIDAR_;
  }
  ~LMS1Collector() { PRINT_INFO("destroy a LMS1xx collector."); };

  int32_t init(char host[], int32_t port);
  void deInit();
  int32_t get_scan_msg(Bgs::LaserScanMsg *scan_msg) override;
  int32_t get_scan_cfg(Bgs::LaserScanMsg *scan_msg) override;

  /// @warning you should not use the function, it is for a 3d lidar
  int32_t get_point_cloud2_cfg(Bgs::PointCloud2Ptr &cloud) override;
  /// @warning you should not use the function, it is for a 3d lidar
  int32_t get_point_cloud2(Bgs::PointCloud2Ptr &cloud) override;

  /*!
   * @brief inner loop to update laser data ( for thread_update_data_ )
   */
  void update_data();
};

/// \brief class LMS1Collector
///
/// it a class for 3d lidar
/// robotsense rs16 / rs32
class RoboSenseCollector : public LidarCollector
{
private:
  std::shared_ptr<rslidar::driver::rslidarDriver> lidar_;
  std::shared_ptr<rslidar::point_cloud::Convert> convertor_;

  std::thread thread_update_data_;
  bool exit_thread_;
  bool init_finished_;
  Bgs::PointCloud2Ptr point_cloud2_;
  LidarCallback3d callback_;

public:
  RoboSenseCollector(rslidar::driver::Config &driver_config,
                     rslidar::rawdata::Config &point_cloud_config,
                     Bgs::PointCloud2Ptr &point_cloud2_ptr,
                     LidarCallback3d callback)
      : lidar_(new rslidar::driver::rslidarDriver(driver_config))
      , convertor_(new rslidar::point_cloud::Convert(point_cloud_config))
      , thread_update_data_(std::bind(&RoboSenseCollector::update_data, this))
      , exit_thread_(false)
      , init_finished_(false)
      , point_cloud2_(point_cloud2_ptr)
      , callback_(callback)
  {
    type_ = _3D_LIDAR_;
    init_finished_ = true;
  }

  ~RoboSenseCollector()
  {
    convertor_.reset();
    lidar_.reset();
    PRINT_INFO("destroy a robosense collector.");
  }

  int32_t init(char host[], int32_t port);
  void deInit();
  /// @warning you should not use the function, it is for a 2d lidar
  int32_t get_scan_msg(Bgs::LaserScanMsg *scan_msg) override;
  /// @warning you should not use the function, it is for a 2d lidar
  int32_t get_scan_cfg(Bgs::LaserScanMsg *scan_msg) override;

  int32_t get_point_cloud2_cfg(Bgs::PointCloud2Ptr &cloud) override;
  int32_t get_point_cloud2(Bgs::PointCloud2Ptr &cloud) override;

  /*!
   * @brief inner loop to update laser data ( for thread_update_data_ )
   */
  void update_data();
};
}
