/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  RSLIDAR 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw RSLIDAR LIDAR packets into useful
 *  formats.
 *
 */
#include "rawdata.h"

namespace hardware
{
namespace rslidar
{
namespace rawdata
{

float VERT_ANGLE[32];
float HORI_ANGLE[32];
float aIntensityCal[1600][32];
int g_ChannelNum[32][51];
float CurvesRate[32];

float temper = 31.0;
int tempPacketNum = 0;
int numOfLasers = 16;
int TEMPERATURE_RANGE = 40;

hardware::rslidar::msgs::RslidarPic pic;

RawData::RawData() {}

void RawData::loadConfigFile(const rawdata::Config &config)
{
  std::string anglePath = config.anglePath;
  std::string curvesPath = config.curvesPath;
  std::string channelPath = config.channelPath;
  std::string curvesRatePath = config.curvesRatePath;
  std::string model = config.model;

  if (model == "RS16") {
    numOfLasers = 16;
  } else if (model == "RS32") {
    numOfLasers = 32;
    TEMPERATURE_RANGE = 50;
  }

  /// 读参数文件 2017-02-27
  FILE *f_inten = fopen(curvesPath.c_str(), "r");
  int loopi = 0;
  int loopj = 0;

  if (!f_inten) {
    PRINT_WARNING_FMT("%s does not exist", curvesPath.c_str());
  } else {
    while (!feof(f_inten)) {
      float a[32];
      loopi++;
      if (loopi > 1600)
        break;
      if (numOfLasers == 16) {
        fscanf(f_inten, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
               &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &a[8],
               &a[9], &a[10], &a[11], &a[12], &a[13], &a[14], &a[15]);
      } else if (numOfLasers == 32) {
        fscanf(f_inten, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"
                        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
               &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &a[8],
               &a[9], &a[10], &a[11], &a[12], &a[13], &a[14], &a[15], &a[16],
               &a[17], &a[18], &a[19], &a[20], &a[21], &a[22], &a[23], &a[24],
               &a[25], &a[26], &a[27], &a[28], &a[29], &a[30], &a[31]);
      }
      for (loopj = 0; loopj < numOfLasers; loopj++) {
        aIntensityCal[loopi - 1][loopj] = a[loopj];
      }
    }
    fclose(f_inten);
  }
  //=============================================================
  FILE *f_angle = fopen(anglePath.c_str(), "r");
  if (!f_angle) {
    PRINT_WARNING_FMT("%s does not exist", anglePath.c_str());
  } else {
    float b[32], d[32];
    int loopk = 0;
    int loopn = 0;
    while (!feof(f_angle)) {
      fscanf(f_angle, "%f,%f\n", &b[loopk], &d[loopk]);
      loopk++;
      if (loopk > (numOfLasers - 1))
        break;
    }
    for (loopn = 0; loopn < numOfLasers; loopn++) {
      VERT_ANGLE[loopn] = b[loopn] / 180 * M_PI;
      HORI_ANGLE[loopn] = d[loopn] * 100;
    }
    fclose(f_angle);
  }

  //=============================================================
  FILE *f_channel = fopen(channelPath.c_str(), "r");
  if (!f_channel) {
    PRINT_WARNING_FMT("%s does not exist", channelPath.c_str());
  } else {
    PRINT_INFO("Loading channelnum corrections file!");
    int loopl = 0;
    int loopm = 0;
    int c[51];
    int tempMode = 1;
    while (!feof(f_channel)) {
      if (numOfLasers == 16) {
        fscanf(f_channel, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%"
                          "d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%"
                          "d,%d,%d,%d,%d,%d,%d\n",
               &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8],
               &c[9], &c[10], &c[11], &c[12], &c[13], &c[14], &c[15], &c[16],
               &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24],
               &c[25], &c[26], &c[27], &c[28], &c[29], &c[30], &c[31], &c[32],
               &c[33], &c[34], &c[35], &c[36], &c[37], &c[38], &c[39], &c[40]);
      } else {
        fscanf(f_channel, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%"
                          "d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%"
                          "d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
               &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8],
               &c[9], &c[10], &c[11], &c[12], &c[13], &c[14], &c[15], &c[16],
               &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24],
               &c[25], &c[26], &c[27], &c[28], &c[29], &c[30], &c[31], &c[32],
               &c[33], &c[34], &c[35], &c[36], &c[37], &c[38], &c[39], &c[40],
               &c[41], &c[42], &c[43], &c[44], &c[45], &c[46], &c[47], &c[48],
               &c[49], &c[50]);
      }
      if (c[1] < 100 || c[1] > 3000) {
        tempMode = 0;
      }
      for (loopl = 0; loopl < TEMPERATURE_RANGE + 1; loopl++) {
        g_ChannelNum[loopm][loopl] = c[tempMode * loopl];
      }
      loopm++;
      if (loopm > (numOfLasers - 1)) {
        break;
      }
    }
    fclose(f_channel);
  }

  FILE *f_curvesRate = fopen(curvesRatePath.c_str(), "r");
  if (!f_curvesRate) {
    PRINT_WARNING_FMT("curves rate file: %s does not exist",
                      curvesRatePath.c_str());
  } else {
    int loopk = 0;
    while (!feof(f_curvesRate)) {
      fscanf(f_curvesRate, "%f\n", &CurvesRate[loopk]);
      loopk++;
      if (loopk > (numOfLasers - 1))
        break;
    }
    fclose(f_curvesRate);
  }
}

/** Set up for on-line operation. */
void RawData::init_setup()
{
  pic.col = 0;
  if (numOfLasers == 16) {
    pic.distance.resize(RS16_DATA_NUMBER_PER_SCAN);
    pic.intensity.resize(RS16_DATA_NUMBER_PER_SCAN);
    pic.azimuthforeachP.resize(RS16_DATA_NUMBER_PER_SCAN);
  } else if (numOfLasers == 32) {
    pic.distance.resize(RS32_DATA_NUMBER_PER_SCAN);
    pic.intensity.resize(RS32_DATA_NUMBER_PER_SCAN);
    pic.azimuthforeachP.resize(RS32_DATA_NUMBER_PER_SCAN);
  }
}

float RawData::pixelToDistance(int pixelValue, int passageway)
{
  float DistanceValue;
  int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;
  if (pixelValue <= g_ChannelNum[passageway][indexTemper]) {
    DistanceValue = 0.0;
  } else {
    DistanceValue = (float)(pixelValue - g_ChannelNum[passageway][indexTemper]);
  }
  return DistanceValue;
}

int RawData::correctAzimuth(float azimuth_f, int passageway)
{
  int azimuth;
  if (azimuth_f > 0.0 && azimuth_f < 3000.0) {
    azimuth_f = azimuth_f + HORI_ANGLE[passageway] + 36000.0f;
  } else {
    azimuth_f = azimuth_f + HORI_ANGLE[passageway];
  }
  azimuth = (int)azimuth_f;
  azimuth %= 36000;

  return azimuth;
}

//------------------------------------------------------------
//校准反射强度值
float RawData::calibrateIntensity(float intensity, int calIdx, int distance)
{
  int algDist;
  int sDist;
  int uplimitDist;
  float realPwr;
  float refPwr;
  float tempInten;

  if (intensity == 0.0) {
    tempInten = 0.0;
    return tempInten;
  }

  int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;
  uplimitDist = g_ChannelNum[calIdx][indexTemper] + 1400;
  realPwr = intensity;

  if ((int)realPwr < 126)
    realPwr = realPwr * 4.0f;
  else if ((int)realPwr >= 126 && (int)realPwr < 226)
    realPwr = (realPwr - 125.0f) * 16.0f + 500.0f;
  else
    realPwr = (realPwr - 225.0f) * 256.0f + 2100.0f;

  sDist = (distance > g_ChannelNum[calIdx][indexTemper])
            ? distance
            : g_ChannelNum[calIdx][indexTemper];
  sDist = (sDist < uplimitDist) ? sDist : uplimitDist;
  // minus the static offset (this data is For the intensity cal useage only)
  algDist = sDist - g_ChannelNum[calIdx][indexTemper];
  // algDist = algDist < 1400? algDist : 1399;
  refPwr = aIntensityCal[algDist][calIdx];
  tempInten = (200 * refPwr) / realPwr;
  tempInten = tempInten * CurvesRate[calIdx];
  tempInten = (int)tempInten > 255 ? 255.0f : tempInten;
  return tempInten;
}

//------------------------------------------------------------
int RawData::isABPacket(int distance)
{
  int ABflag = 0;
  if ((distance & 32768) != 0) {
    ABflag = 1; // B
  } else {
    ABflag = 0; // A
  }
  return ABflag;
}

//------------------------------------------------------------
float RawData::computeTemperature(unsigned char bit1, unsigned char bit2)
{
  float Temp;
  float bitneg = bit2 & 128;  // 10000000
  float highbit = bit2 & 127; // 01111111
  float lowbit = bit1 >> 3;
  if (bitneg == 128) {
    Temp = -1 * (highbit * 32 + lowbit) * 0.0625f;
  } else {
    Temp = (highbit * 32 + lowbit) * 0.0625f;
  }

  return Temp;
}

//------------------------------------------------------------
int RawData::estimateTemperature(float Temper)
{
  int temp = (int)floor(Temper + 0.5);
  if (temp < TEMPERATURE_MIN) {
    temp = TEMPERATURE_MIN;
  } else if (temp > TEMPERATURE_MIN + TEMPERATURE_RANGE) {
    temp = TEMPERATURE_MIN + TEMPERATURE_RANGE;
  }

  return temp;
}
//------------------------------------------------------------

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pointcloud shared pointer to point cloud (points are appended)
 *  @param finish_packets_parse
 */
void RawData::unpack(const msgs::RslidarPacket &pkt,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,
                     bool finish_packets_parse)
{
  if (numOfLasers == 32) {
    unpack_RS32(pkt, pointcloud, finish_packets_parse);
    return;
  }
  float azimuth; // 0.01 dgree
  float intensity;
  float azimuth_diff;
  float azimuth_corrected_f;
  int azimuth_corrected;

  const raw_packet_t *raw = (const raw_packet_t *)&pkt.data[42];

  for (int block = 0; block < BLOCKS_PER_PACKET;
       block++) // 1 packet:12 data blocks
  {

    if (UPPER_BANK != raw->blocks[block].header) {
      PRINT_ERROR("skipping RSLIDAR DIFOP packet");
      break;
    }

    if (tempPacketNum < 20000
        && tempPacketNum
             > 0) // update temperature information per 20000 packets
    {
      tempPacketNum++;
    } else {
      temper = computeTemperature(pkt.data[38], pkt.data[39]);
      // ROS_INFO_STREAM("Temp is: " << temper);
      tempPacketNum = 1;
    }

    azimuth = (float)(256 * raw->blocks[block].rotation_1
                      + raw->blocks[block].rotation_2);

    if (block < (BLOCKS_PER_PACKET - 1)) // 12
    {
      int azi1, azi2;
      azi1 = 256 * raw->blocks[block + 1].rotation_1
             + raw->blocks[block + 1].rotation_2;
      azi2 =
        256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);

      // Ingnore the block if the azimuth change abnormal
      if (azimuth_diff <= 0.0 || azimuth_diff > 75.0) {
        continue;
      }

    } else {
      int azi1, azi2;
      azi1 =
        256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      azi2 = 256 * raw->blocks[block - 1].rotation_1
             + raw->blocks[block - 1].rotation_2;
      azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);

      // Ingnore the block if the azimuth change abnormal
      if (azimuth_diff <= 0.0 || azimuth_diff > 75.0) {
        continue;
      }
    }

    for (int firing = 0, k = 0; firing < RS16_FIRINGS_PER_BLOCK; firing++) // 2
    {
      for (int dsr = 0; dsr < RS16_SCANS_PER_FIRING;
           dsr++, k += RAW_SCAN_SIZE) // 16   3
      {
        int point_count =
          pic.col * SCANS_PER_BLOCK + dsr + RS16_SCANS_PER_FIRING * firing;
        azimuth_corrected_f =
          azimuth + (azimuth_diff * ((dsr * RS16_DSR_TOFFSET)
                                     + (firing * RS16_FIRING_TOFFSET))
                     / RS16_BLOCK_TDURATION);
        azimuth_corrected = ((int)round(azimuth_corrected_f))
                            % 36000; // convert to integral value...
        pic.azimuthforeachP[point_count] = azimuth_corrected;

        union two_bytes tmp;
        tmp.bytes[1] = raw->blocks[block].data[k];
        tmp.bytes[0] = raw->blocks[block].data[k + 1];
        int distance = tmp.uint;

        // read intensity
        intensity = raw->blocks[block].data[k + 2];
        intensity = calibrateIntensity(intensity, dsr, distance);

        float distance2 = pixelToDistance(distance, dsr);
        distance2 = distance2 * DISTANCE_RESOLUTION;

        pic.distance[point_count] = distance2;
        pic.intensity[point_count] = intensity;
      }
    }
    // pic.azimuth[pic.col] = azimuth;
    pic.col++;
  }

  if (finish_packets_parse) {
    // ROS_INFO_STREAM("***************: "<<pic.col);
    pointcloud->clear();
    pointcloud->height = RS16_SCANS_PER_FIRING;
    pointcloud->width = 2 * pic.col;
    pointcloud->is_dense = false;
    pointcloud->resize(pointcloud->height * pointcloud->width);
    for (uint32_t block_num = 0; block_num < pic.col; block_num++) {

      for (int firing = 0; firing < RS16_FIRINGS_PER_BLOCK; firing++) {
        for (int dsr = 0; dsr < RS16_SCANS_PER_FIRING; dsr++) {
          int point_count =
            block_num * SCANS_PER_BLOCK + dsr + RS16_SCANS_PER_FIRING * firing;
          float dis = pic.distance[point_count];
          float arg_horiz = pic.azimuthforeachP[point_count] / 18000 * M_PI;
          float arg_vert = VERT_ANGLE[dsr];
          pcl::PointXYZI point;
          if (dis > DISTANCE_MAX || dis < DISTANCE_MIN) // invalid data
          {
            point.x = NAN;
            point.y = NAN;
            point.z = NAN;
            point.intensity = 0;
            pointcloud->at(2 * block_num + firing, dsr) = point;
          } else {
            // If you want to fix the rslidar Y aixs to the front side of the
            // cable, please use the two line below
            // point.x = dis * cos(arg_vert) * sin(arg_horiz);
            // point.y = dis * cos(arg_vert) * cos(arg_horiz);

            // If you want to fix the rslidar X aixs to the front side of the
            // cable, please use the two line below
            point.y = -dis * cos(arg_vert) * sin(arg_horiz);
            point.x = dis * cos(arg_vert) * cos(arg_horiz);
            point.z = dis * sin(arg_vert);
            point.intensity = pic.intensity[point_count];
            pointcloud->at(2 * block_num + firing, dsr) = point;
          }
        }
      }
    }
    init_setup();
    pic.header.stamp = pkt.stamp;
  }
}

void RawData::unpack_RS32(const msgs::RslidarPacket &pkt,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,
                          bool finish_packets_parse)
{
  float azimuth; // 0.01 dgree
  float intensity;
  float azimuth_diff;
  float azimuth_corrected_f;
  int azimuth_corrected;

  const raw_packet_t *raw = (const raw_packet_t *)&pkt.data[42];

  for (int block = 0; block < BLOCKS_PER_PACKET;
       block++) // 1 packet:12 data blocks
  {

    if (UPPER_BANK != raw->blocks[block].header) {
      PRINT_ERROR("skipping RSLIDAR DIFOP packet");
      break;
    }

    if (tempPacketNum < 20000
        && tempPacketNum
             > 0) // update temperature information per 20000 packets
    {
      tempPacketNum++;
    } else {
      temper = computeTemperature(pkt.data[38], pkt.data[39]);
      // ROS_INFO_STREAM("Temp is: " << temper);
      tempPacketNum = 1;
    }

    azimuth = (float)(256 * raw->blocks[block].rotation_1
                      + raw->blocks[block].rotation_2);

    if (block < (BLOCKS_PER_PACKET - 1)) // 12
    {
      int azi1, azi2;
      azi1 = 256 * raw->blocks[block + 1].rotation_1
             + raw->blocks[block + 1].rotation_2;
      azi2 =
        256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);

      // Ingnore the block if the azimuth change abnormal
      if (azimuth_diff <= 0.0 || azimuth_diff > 25.0) {
        continue;
      }
    } else {
      int azi1, azi2;
      azi1 =
        256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      azi2 = 256 * raw->blocks[block - 1].rotation_1
             + raw->blocks[block - 1].rotation_2;
      azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);

      // Ingnore the block if the azimuth change abnormal
      if (azimuth_diff <= 0.0 || azimuth_diff > 25.0) {
        continue;
      }
    }

    // Estimate the type of packet
    union two_bytes tmp_flag;
    tmp_flag.bytes[1] = raw->blocks[block].data[0];
    tmp_flag.bytes[0] = raw->blocks[block].data[1];
    int ABflag = isABPacket(tmp_flag.uint);

    int k = 0;
    int index;
    for (int dsr = 0; dsr < RS32_SCANS_PER_FIRING * RS32_FIRINGS_PER_BLOCK;
         dsr++, k += RAW_SCAN_SIZE) // 16   3
    {
      if (ABflag == 1 && dsr < 16) {
        index = k + 48;
      } else if (ABflag == 1 && dsr >= 16) {
        index = k - 48;
      } else {
        index = k;
      }

      int point_count = pic.col * SCANS_PER_BLOCK + dsr;
      int dsr_temp;
      if (dsr >= 16) {
        dsr_temp = dsr - 16;
      } else {
        dsr_temp = dsr;
      }
      azimuth_corrected_f =
        azimuth + (azimuth_diff * ((dsr_temp * RS32_DSR_TOFFSET))
                   / RS32_BLOCK_TDURATION);
      azimuth_corrected = correctAzimuth(azimuth_corrected_f, dsr);
      pic.azimuthforeachP[point_count] = azimuth_corrected;

      union two_bytes tmp;
      tmp.bytes[1] = raw->blocks[block].data[index];
      tmp.bytes[0] = raw->blocks[block].data[index + 1];
      int ab_flag_in_block = isABPacket(tmp.uint);
      int distance = tmp.uint - ab_flag_in_block * 32768;

      // read intensity
      intensity = (float)raw->blocks[block].data[index + 2];
      intensity = calibrateIntensity(intensity, dsr, distance);

      float distance2 = pixelToDistance(distance, dsr);
      distance2 = distance2 * DISTANCE_RESOLUTION;

      pic.distance[point_count] = distance2;
      pic.intensity[point_count] = intensity;
    }
    // pic.azimuth[pic.col] = azimuth;
    pic.col++;
  }

  if (finish_packets_parse) {
    // ROS_INFO_STREAM("***************: "<<pic.col);
    pointcloud->clear();
    pointcloud->height = RS32_SCANS_PER_FIRING;
    pointcloud->width = pic.col;
    pointcloud->is_dense = false;
    pointcloud->resize(pointcloud->height * pointcloud->width);
    for (uint32_t block_num = 0; block_num < pic.col; block_num++) {

      for (int dsr = 0; dsr < RS32_SCANS_PER_FIRING * RS32_FIRINGS_PER_BLOCK;
           dsr++) {
        int point_count = block_num * SCANS_PER_BLOCK + dsr;
        float dis = pic.distance[point_count];
        float arg_horiz = pic.azimuthforeachP[point_count] / 18000 * M_PI;
        float intensity = pic.intensity[point_count];
        float arg_vert = VERT_ANGLE[dsr];
        pcl::PointXYZI point;
        if (dis > DISTANCE_MAX || dis < DISTANCE_MIN) // invalid data
        {
          // ROS_INFO_STREAM("***************: "<<dis);
          point.x = NAN;
          point.y = NAN;
          point.z = NAN;
          point.intensity = 0;
          pointcloud->at(block_num, dsr) = point;
        } else {
          // If you want to fix the rslidar Y aixs to the front side of the
          // cable, please use the two line below
          // point.x = dis * cos(arg_vert) * sin(arg_horiz);
          // point.y = dis * cos(arg_vert) * cos(arg_horiz);

          // If you want to fix the rslidar X aixs to the front side of the
          // cable, please use the two line below
          point.y = -dis * cos(arg_vert) * sin(arg_horiz);
          point.x = dis * cos(arg_vert) * cos(arg_horiz);
          point.z = dis * sin(arg_vert);
          point.intensity = intensity;
          pointcloud->at(block_num, dsr) = point;
        }
      }
    }
    init_setup();
    pic.header.stamp = pkt.stamp;
  }
}

} // namespace rs_pointcloud
}
}
