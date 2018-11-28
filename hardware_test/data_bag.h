/*!
 * @file data_bag.h
 * @brief defines and functions for data bag.
 *
 * @author edward.liu
 * @date 2018-10-08
 */

#pragma once

#include "struct_defines.h"
#include <sys/types.h>
#include <zlib.h>

#define BAG_LASER_AVAILABLE 0x00000001
#define BAG_IMU_AVAILABLE 0x00000002
#define BAG_ODOM_AVAILABLE 0x00000004
#define BAG_GPS_AVAILABLE 0x00000008
#define BAG_MTR_AVAILABLE 0x00000010
#define BAG_POINT_CLOUD_AVAILABLE 0x00000020

// for simu bags (bgs_data_bag)
#define BGS_BAG_START_NUM 0xAABBCCDD
#define BGS_BAG_END_NUM 0x11335577

namespace hardware
{

/*!
 * @namespace hardware::databag
 * @brief defines and functions for data bag.
 *
 * @details
 * This databag is something like a ROS bag, but it is much simpler
 *
 * the bag is conbined with many frames:
 * - start frame
 * - data frames
 * - end frame
 *
 * every frame has a @a start_ and an @a end_ to ensure its completeness, as
 * we can see in the struct DataFrame, @a type_ indicates the
 * content in the frame.
 *
 * a data frame can includes all kinds of sensor data in the project:
 * - laser scan
 * - point cloud
 * - gps
 * - imu
 *
 * and if it contains laser scan or point cloud, it should be compressed when
 * write into a data bag file, and be uncompressed when read from a bag file,
 * we use <a href="https://zlib.net/">Zlib</a> to achieve these functions.
 */
namespace databag
{

enum FrameType {
  kStartFrame,
  kAllSensorFrame,
  kOnlyImuFrame,
  kOnlyPointCloudFrame,
  kOnlyLaserScanFrame,
  kOnlyGpsFrame,
  kEndFrame,
  kTypeCount
};

struct DataFrame {
  uint32_t start_ = BGS_BAG_START_NUM;

  FrameType type_;

  void *data_;

  uint32_t end_ = BGS_BAG_END_NUM;
};

struct StartData {
  BgsTime stamp_;                 ///< time stamp of the start of the bag
  int32_t all_available_sensors_; ///< conbined with the macros define above

  int32_t reserved[9]; ///< reserved for future use
};

struct SensorData {
  int32_t available_sensors_;
  uLongf compressed_size_;
  uLongf uncompressed_size_;
  Byte *compressed_data_;
  Byte *uncompressed_data_;
};

struct EndData {
  int32_t all_frame_count; ///< count of data frames, exclude start and end
  int32_t reserved[3];     ///< reserved for future use
};

/*!
 * @brief write a frame to the bag file
 *
 * - step1, write the @a start_ and @a type into the bag
 * - step2, if it is a start or end frame, then directly write the data into the
 * bag file, or if it is a sensor frame, the data will be compressed before
 * writing into the bag
 * - we write the size of data ahead of the @a data_.
 * - finally, write the @a end_;
 *
 * @param fout bag file
 * @param frame frame data
 *
 * @return
 */
int32_t write_to_data_bag(std::ofstream &fout, DataFrame &frame);

/*!
 * @brief
 *
 * since the bag file is usually large, we use mmap to open the bag
 * file instead of fopen or open
 *
 * @param filename
 * @param size send out the size of the file
 *
 * @return the pointer got from mmap
 *
 * @note you should close the bag file when done with reading, and you can use
 * the function: close_bag_file to do it
 */
void *open_bag_file(const char *filename, int32_t *size);

/*!
 * @brief use this after you open the bag file
 *
 * @param data the pointer got from open_bag_file
 * @param size the size got from open_bag_file
 *
 * @return succeed to unmap the memory or not
 */
int32_t close_bag_file(Byte *data, int32_t size);

/*!
 * @brief read every single frame
 *
 * @param data the begining of the memory mapped to the bag
 * @param offset the position we want to start reading
 * @param type frame type: start/sensor.../end
 * @param start_data if the type is kStartFrame, you can get the data from this
 * pointer
 * @param sensor_data if the type is kAllSensorFrame/..., you can get the data
 * from this pointer
 * @param end_data if the type is kEndFrame, you can get the data from this
 * pointer
 *
 * @return
 * - if you succeed to read the frame, you will get a new offset for next frame;
 * - or you will get a -1 if failed to read the frame
 *
 *
 */
int32_t read_single_frame(Byte *data, int32_t offset, FrameType *type,
                          StartData *start_data, SensorData *sensor_data,
                          EndData *end_data);
} // namespace databag
} // namespace hardware
