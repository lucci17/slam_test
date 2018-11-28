#include "data_bag.h"
#include "macro_defines.h"
#include <fcntl.h>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <sys/mman.h>
#include <zlib.h>

namespace hardware
{
namespace databag
{

int32_t write_to_data_bag(std::ofstream &fout, DataFrame &frame)
{
  if (frame.data_ == NULL) {
    PRINT_ERROR("the data is empty!");
    return -1;
  }

  // 把目标数据放到string里，然后压缩之后放到文件里去
  // 这只是个暂时的方案，后面会在各个线程里面实现这个数据的保存功能 TODO
  // 先利用 zlib 将数据压缩后再写入到文件中，可以明显降低保存的文件大小

  int32_t size = 0;
  void *data = NULL;

  fout.write((char *)(&frame.start_), sizeof(frame.start_));
  fout.write((char *)(&frame.type_), sizeof(frame.type_));

  switch (frame.type_) {
  case kStartFrame:
    size = sizeof(StartData);
    data = frame.data_;
    break;

  // 这里传感器的压缩方式写文件方式类似，都是压缩后写入
  // 唯一的区别是压缩的数据不同，All
  // Sensor是压缩整个共享内存，而其他的是针对单独内存块的压缩
  // 但这个区别并不体现在压缩后的数据上，而是体现在压缩时
  case kOnlyImuFrame:
  case kOnlyPointCloudFrame:
  case kOnlyLaserScanFrame:
  case kOnlyGpsFrame:
  case kAllSensorFrame: {
    size = ((SensorData *)frame.data_)->compressed_size_;
    data = ((SensorData *)frame.data_)->compressed_data_;

    int32_t available_sensors = ((SensorData *)frame.data_)->available_sensors_;
    // 判断当前传入的数据和设定的属性是否匹配
    if (frame.type_ == kOnlyImuFrame && available_sensors != BAG_IMU_AVAILABLE)
      PRINT_WARNING("it should be a only imu frame!");
    if (frame.type_ == kOnlyPointCloudFrame
        && available_sensors != BAG_POINT_CLOUD_AVAILABLE)
      PRINT_WARNING("it should be a only point cloud frame!");
    if (frame.type_ == kOnlyLaserScanFrame
        && available_sensors != BAG_LASER_AVAILABLE)
      PRINT_WARNING("it should be a only laser scan frame!");
    if (frame.type_ == kOnlyGpsFrame && available_sensors != BAG_GPS_AVAILABLE)
      PRINT_WARNING("it should be a only gps frame!");

    fout.write((char *)(&available_sensors), sizeof(int32_t));
    fout.write((char *)(&size), sizeof(int32_t));
  } break;

  case kEndFrame:
    size = sizeof(EndData);
    data = frame.data_;
    break;

  default:
    PRINT_ERROR("Wrong bag type!");
    return -1;
  }

  fout.write((char *)data, size);
  fout.write((char *)(&frame.end_), sizeof(frame.end_));
  return 0;
}

void *open_bag_file(const char *filename, int32_t *size)
{
  if (filename == NULL || size == NULL) {
    PRINT_ERROR("filename is empty!");
    return MAP_FAILED;
  }

  int fd = open(filename, O_RDONLY);
  if (fd < 0) {
    PRINT_ERROR("failed to open file!");
    return MAP_FAILED;
  }
  PRINT_INFO_FMT("open bag file: %s", filename);

  int32_t file_size = lseek(fd, 0, SEEK_END);
  void *data = mmap(NULL, file_size, PROT_READ, MAP_PRIVATE, fd, 0);
  if (data == MAP_FAILED || data == NULL) {
    PRINT_ERROR("failed to mmap file to memory!");
    close(fd);
    return MAP_FAILED;
  }

  *size = file_size;

  close(fd);
  return data;
}

int32_t close_bag_file(Byte *data, int32_t size)
{
  if (munmap(data, size) == 0) {
    PRINT_INFO("unmap succeed!");
    return 0;
  } else {
    PRINT_ERROR("unmap error!");
    return -1;
  }

  data = NULL;
  return 0;
}

// 返回 -1 则读取失败
// 返回值大于 0 则成功，返回的是当前这一帧结束的 offset（下一帧的起始 offset）
int32_t read_single_frame(Byte *data, int32_t offset, FrameType *type,
                          StartData *start_data, SensorData *sensor_data,
                          EndData *end_data)
{
  if (!data || offset < 0 || !type)
    return -1;

  // start num
  if (*(uint32_t *)(data + offset) != BGS_BAG_START_NUM) {
    PRINT_ERROR("it is not a start!");
    return -1;
  }
  offset += sizeof(uint32_t);

  // type
  *type = *(FrameType *)(data + offset); // frame type
  offset += sizeof(FrameType);

  // internal data
  switch (*type) {
  case kStartFrame:
    if (!start_data) {
      PRINT_ERROR("start data is null");
      return -1;
    }
    *start_data = *(StartData *)(data + offset); // start data
    offset += sizeof(StartData);
    break;

  case kOnlyImuFrame:
  case kOnlyPointCloudFrame:
  case kOnlyLaserScanFrame:
  case kOnlyGpsFrame:
  case kAllSensorFrame:
    if (!sensor_data || !(sensor_data->uncompressed_data_)) {
      PRINT_ERROR("sensor data is null");
      return -1;
    }
    sensor_data->available_sensors_ = *(int32_t *)(data + offset);
    offset += sizeof(int32_t);
    sensor_data->compressed_size_ = *(int32_t *)(data + offset);
    offset += sizeof(int32_t);
    if (Z_OK == uncompress(sensor_data->uncompressed_data_,
                           &sensor_data->uncompressed_size_, (data + offset),
                           sensor_data->compressed_size_)) {
      offset += sensor_data->compressed_size_;
    } else {
      PRINT_ERROR("uncompress failed!");
      return -1;
    }

    break;

  default:
    if (!end_data) {
      PRINT_ERROR("end data is null");
      return -1;
    }
    *end_data = *(EndData *)(data + offset);
    offset += sizeof(EndData); // end data
    break;
  }

  // end num
  if (*(uint32_t *)(data + offset) != BGS_BAG_END_NUM) {
    PRINT_ERROR("it is not a complete frame!");
    return -1;
  }
  offset += sizeof(uint32_t);

  return offset;
}
} /// namespace databag

} /// namespace hardware
