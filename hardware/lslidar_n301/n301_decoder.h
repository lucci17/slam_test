#ifndef N301_DECODER_H
#define N301_DECODER_H

#include "n301_struct.h"
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <linux/types.h>
#include <list>
#include <mutex>

#define DEG_TO_RAD 0.017453292
#define RAD_TO_DEG 57.29577951

// Raw N301 packet constants and structures.
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = // 32 * 3
  (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

// According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
// valid packets with readings up to 130.0.
static const double DISTANCE_MAX = 130.0;        /**< meters */
static const double DISTANCE_RESOLUTION = 0.002; /**< meters */
static const double DISTANCE_MAX_UNITS =
  (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0);

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const int FIRINGS_PER_BLOCK = 2;
static const int SCANS_PER_FIRING = 16;
static const double BLOCK_TDURATION = 110.592; // [µs]
static const double DSR_TOFFSET = 2.304;       // [µs]
static const double FIRING_TOFFSET = 55.296;   // [µs]

static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = // 32 * 12
  (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);
static const int FIRINGS_PER_PACKET = // 2 * 12
  FIRINGS_PER_BLOCK * BLOCKS_PER_PACKET;

// Pre-compute the sine and cosine for the altitude angles.
static const double scan_altitude[16] = {
  -0.2617993877991494,  0.017453292519943295, -0.22689280275926285,
  0.05235987755982989,  -0.19198621771937624, 0.08726646259971647,
  -0.15707963267948966, 0.12217304763960307,  -0.12217304763960307,
  0.15707963267948966,  -0.08726646259971647, 0.19198621771937624,
  -0.05235987755982989, 0.22689280275926285,  -0.017453292519943295,
  0.2617993877991494
};

static const double cos_scan_altitude[16] = {
  cos(scan_altitude[0]),  cos(scan_altitude[1]),  cos(scan_altitude[2]),
  cos(scan_altitude[3]),  cos(scan_altitude[4]),  cos(scan_altitude[5]),
  cos(scan_altitude[6]),  cos(scan_altitude[7]),  cos(scan_altitude[8]),
  cos(scan_altitude[9]),  cos(scan_altitude[10]), cos(scan_altitude[11]),
  cos(scan_altitude[12]), cos(scan_altitude[13]), cos(scan_altitude[14]),
  cos(scan_altitude[15]),
};

static const double sin_scan_altitude[16] = {
  sin(scan_altitude[0]),  sin(scan_altitude[1]),  sin(scan_altitude[2]),
  sin(scan_altitude[3]),  sin(scan_altitude[4]),  sin(scan_altitude[5]),
  sin(scan_altitude[6]),  sin(scan_altitude[7]),  sin(scan_altitude[8]),
  sin(scan_altitude[9]),  sin(scan_altitude[10]), sin(scan_altitude[11]),
  sin(scan_altitude[12]), sin(scan_altitude[13]), sin(scan_altitude[14]),
  sin(scan_altitude[15]),
};

typedef struct {
  float distance;
  float intensity;
} point_struct;

class N301_decoder
{
public:
  static N301_decoder *instance();
  explicit N301_decoder();
  ~N301_decoder();

public:
  inline bool isDataReady() { return sweep_data_ready; }

  inline void setDataReady(bool is_ready) { sweep_data_ready = is_ready; }

  N301_packet *msg;
  std::list<N301_packet> msg_list;
  N301_lidar *lidar;
  N301_lidar *ready_data;

  void getOnePacket();

private:
  union TwoBytes {
    uint16_t distance;
    uint8_t bytes[2];
  };

  struct RawBlock {
    uint16_t header;   ///< UPPER_BANK or LOWER_BANK
    uint16_t rotation; ///< 0-35999, divide by 100 to get degrees
    uint8_t data[BLOCK_DATA_SIZE];
  };

  struct RawPacket {
    RawBlock blocks[BLOCKS_PER_PACKET];
    uint32_t time_stamp;
    uint8_t factory[2];
  };

  struct Firing {
    // Azimuth associated with the first shot within this firing.
    double firing_azimuth;
    double azimuth[SCANS_PER_FIRING];
    double distance[SCANS_PER_FIRING];
    double intensity[SCANS_PER_FIRING];
  };

  // Check if a point is in the required range.
  inline bool isPointInRange(const double &distance)
  {
    return (distance >= min_range && distance <= max_range);
  }

  inline double rawAzimuthToDouble(const uint16_t &raw_azimuth)
  {
    // According to the user manual,
    // azimuth = raw_azimuth / 100.0;
    return static_cast<double>(raw_azimuth) / 100.0 * DEG_TO_RAD;
  }

  // Configuration parameters
  double min_range;
  double max_range;
  double frequency;

  double cos_azimuth_table[6300];
  double sin_azimuth_table[6300];

  bool is_first_sweep;
  bool sweep_data_ready;
  double last_azimuth;
  double sweep_start_time;
  double packet_start_time;
  Firing firings[FIRINGS_PER_PACKET];

  boost::shared_ptr<N301_sweep> sweep_data;
  std::mutex mutex;

private:
  bool checkPacketValidity(const RawPacket *packet);
  void decodePacket(const RawPacket *packet);

  void saveOneSweep();
  bool initialize();
};

#endif // N301_DECODER_H
