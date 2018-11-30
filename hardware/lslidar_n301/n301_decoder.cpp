#include "n301_decoder.h"
#include <iostream>

N301_decoder *N301_decoder::instance()
{
  static N301_decoder obj;
  return &obj;
}

N301_decoder::N301_decoder()
    : min_range(0.)
    , max_range(100.)
    , is_first_sweep(true)
    , sweep_data_ready(false)
    , last_azimuth(0.)
    , sweep_start_time(0.)
    , packet_start_time(0.)
    , sweep_data(new N301_sweep())
{
  msg = new N301_packet();
  lidar = new N301_lidar();
  ready_data = new N301_lidar();

  initialize();
}

N301_decoder::~N301_decoder()
{
  delete msg;
  delete lidar;
}

bool N301_decoder::initialize()
{
  // Create the sin and cos table for different azimuth values.
  double angle = 0.;
  for (size_t i = 0; i < 6300; ++i) {
    angle = static_cast<double>(i) / 1000.0;
    cos_azimuth_table[i] = cos(angle);
    sin_azimuth_table[i] = sin(angle);
  }

  return true;
}

void N301_decoder::getOnePacket()
{
  // Convert the msg to the raw packet type.
  const RawPacket *raw_packet = (const RawPacket *)(&(msg->data[0]));

  // Check if the packet is valid
  if (!checkPacketValidity(raw_packet))
    return;

  // Decode the packet
  decodePacket(raw_packet);

  // Find the start of a new revolution
  // If there is one, new_sweep_start will be the index of the start firing,
  // otherwise, new_sweep_start will be FIRINGS_PER_PACKET.
  size_t new_sweep_start = 0;
  do {
    if (firings[new_sweep_start].firing_azimuth < last_azimuth)
      break;
    else {
      last_azimuth = firings[new_sweep_start].firing_azimuth;
      ++new_sweep_start;
    }
  } while (new_sweep_start < FIRINGS_PER_PACKET);

  // The first sweep may not be complete. So, the firings with
  // the first sweep will be discarded. We will wait for the
  // second sweep in order to find the 0 azimuth angle.
  size_t start_fir_idx = 0;
  size_t end_fir_idx = new_sweep_start;
  if (is_first_sweep && new_sweep_start == FIRINGS_PER_PACKET) {
    // The first sweep has not ended yet.
    return;
  } else {
    if (is_first_sweep) {
      is_first_sweep = false;
      start_fir_idx = new_sweep_start;
      end_fir_idx = FIRINGS_PER_PACKET;
    }
  }

  for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
    for (size_t scan_idx = 0; scan_idx < SCANS_PER_FIRING; ++scan_idx) {
      // Check if the point is valid.
      if (!isPointInRange(firings[fir_idx].distance[scan_idx]))
        continue;

      // Convert the point to xyz coordinate
      size_t table_idx =
        floor(firings[fir_idx].azimuth[scan_idx] * 1000.0 + 0.5);
      // cout << table_idx << endl;
      double cos_azimuth = cos_azimuth_table[table_idx];
      double sin_azimuth = sin_azimuth_table[table_idx];

      // double x = firings[fir_idx].distance[scan_idx] *
      //  cos_scan_altitude[scan_idx] * sin(firings[fir_idx].azimuth[scan_idx]);
      // double y = firings[fir_idx].distance[scan_idx] *
      //  cos_scan_altitude[scan_idx] * cos(firings[fir_idx].azimuth[scan_idx]);
      // double z = firings[fir_idx].distance[scan_idx] *
      //  sin_scan_altitude[scan_idx];

      double x = firings[fir_idx].distance[scan_idx]
                 * cos_scan_altitude[scan_idx] * sin_azimuth;
      double y = firings[fir_idx].distance[scan_idx]
                 * cos_scan_altitude[scan_idx] * cos_azimuth;
      double z =
        firings[fir_idx].distance[scan_idx] * sin_scan_altitude[scan_idx];

      double x_coord = y;
      double y_coord = -x;
      double z_coord = z;

      // Compute the time of the point
      double time =
        packet_start_time + FIRING_TOFFSET * fir_idx + DSR_TOFFSET * scan_idx;

      // Remap the index of the scan
      int remapped_scan_idx =
        scan_idx % 2 == 0 ? scan_idx / 2 : scan_idx / 2 + 8;
      sweep_data->scans[remapped_scan_idx].points.push_back(N301_point());

      N301_point &new_point = // new_point 为push_back最后一个的引用
        sweep_data->scans[remapped_scan_idx]
          .points[sweep_data->scans[remapped_scan_idx].points.size() - 1];

      // Pack the data into point msg
      new_point.time = time;
      new_point.x = x_coord;
      new_point.y = y_coord;
      new_point.z = z_coord;
      new_point.azimuth = firings[fir_idx].azimuth[scan_idx];
      new_point.distance = firings[fir_idx].distance[scan_idx];
      new_point.intensity = firings[fir_idx].intensity[scan_idx];
    }
  }

  packet_start_time += FIRING_TOFFSET * (end_fir_idx - start_fir_idx);

  // A new sweep begins
  if (end_fir_idx != FIRINGS_PER_PACKET) {
    // save one sweep data
    mutex.lock();
    saveOneSweep();
    mutex.unlock();

    sweep_data = boost::shared_ptr<N301_sweep>(new N301_sweep());
    // Prepare the next revolution
    packet_start_time = 0.0;
    last_azimuth = firings[FIRINGS_PER_PACKET - 1].firing_azimuth;

    start_fir_idx = end_fir_idx;
    end_fir_idx = FIRINGS_PER_PACKET;

    for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
      for (size_t scan_idx = 0; scan_idx < 1; ++scan_idx) {
        // Check if the point is valid.
        if (!isPointInRange(firings[fir_idx].distance[scan_idx]))
          continue;

        // Convert the point to xyz coordinate
        size_t table_idx =
          floor(firings[fir_idx].azimuth[scan_idx] * 1000.0 + 0.5);
        // cout << table_idx << endl;
        double cos_azimuth = cos_azimuth_table[table_idx];
        double sin_azimuth = sin_azimuth_table[table_idx];

        // double x = firings[fir_idx].distance[scan_idx] *
        //  cos_scan_altitude[scan_idx] *
        //  sin(firings[fir_idx].azimuth[scan_idx]);
        // double y = firings[fir_idx].distance[scan_idx] *
        //  cos_scan_altitude[scan_idx] *
        //  cos(firings[fir_idx].azimuth[scan_idx]);
        // double z = firings[fir_idx].distance[scan_idx] *
        //  sin_scan_altitude[scan_idx];

        double x = firings[fir_idx].distance[scan_idx]
                   * cos_scan_altitude[scan_idx] * sin_azimuth;
        double y = firings[fir_idx].distance[scan_idx]
                   * cos_scan_altitude[scan_idx] * cos_azimuth;
        double z =
          firings[fir_idx].distance[scan_idx] * sin_scan_altitude[scan_idx];

        double x_coord = y;
        double y_coord = -x;
        double z_coord = z;

        // Compute the time of the point
        double time = packet_start_time
                      + FIRING_TOFFSET * (fir_idx - start_fir_idx)
                      + DSR_TOFFSET * scan_idx;

        // Remap the index of the scan
        int remapped_scan_idx =
          scan_idx % 2 == 0 ? scan_idx / 2 : scan_idx / 2 + 8;
        sweep_data->scans[remapped_scan_idx].points.push_back(N301_point());
        N301_point &new_point =
          sweep_data->scans[remapped_scan_idx]
            .points[sweep_data->scans[remapped_scan_idx].points.size() - 1];

        // Pack the data into point msg
        new_point.time = time;
        new_point.x = x_coord;
        new_point.y = y_coord;
        new_point.z = z_coord;
        new_point.azimuth = firings[fir_idx].azimuth[scan_idx];
        new_point.distance = firings[fir_idx].distance[scan_idx];
        new_point.intensity = firings[fir_idx].intensity[scan_idx];
      }
    }

    packet_start_time += FIRING_TOFFSET * (end_fir_idx - start_fir_idx);
  }
}

bool N301_decoder::checkPacketValidity(const N301_decoder::RawPacket *packet)
{
  for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
    if (packet->blocks[blk_idx].header != UPPER_BANK) {
      std::cout << "Skip invalid VLP-16 packet: block" << blk_idx << "header is"
                << packet->blocks[blk_idx].header << std::endl;
      return false;
    }
  }
  return true;
}

void N301_decoder::decodePacket(const RawPacket *packet)
{
  // Compute the azimuth angle for each firing.
  for (size_t fir_idx = 0; fir_idx < FIRINGS_PER_PACKET; fir_idx += 2) {
    size_t blk_idx = fir_idx / 2;
    firings[fir_idx].firing_azimuth =
      rawAzimuthToDouble(packet->blocks[blk_idx].rotation);
  }

  // Interpolate the azimuth values
  for (size_t fir_idx = 1; fir_idx < FIRINGS_PER_PACKET; fir_idx += 2) {
    size_t lfir_idx = fir_idx - 1;
    size_t rfir_idx = fir_idx + 1;

    double azimuth_diff;
    if (fir_idx == FIRINGS_PER_PACKET - 1) {
      lfir_idx = fir_idx - 3;
      rfir_idx = fir_idx - 1;
    }

    azimuth_diff =
      firings[rfir_idx].firing_azimuth - firings[lfir_idx].firing_azimuth;
    azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 2 * M_PI : azimuth_diff;

    if (fir_idx == FIRINGS_PER_PACKET - 1) {
      firings[fir_idx].firing_azimuth =
        firings[fir_idx - 1].firing_azimuth + azimuth_diff / 2.0;
    } else {
      firings[fir_idx].firing_azimuth =
        firings[fir_idx - 1].firing_azimuth + azimuth_diff / 2.0;
    }

    firings[fir_idx].firing_azimuth =
      firings[fir_idx].firing_azimuth > 2 * M_PI
        ? firings[fir_idx].firing_azimuth - 2 * M_PI
        : firings[fir_idx].firing_azimuth;
  }

  // Fill in the distance and intensity for each firing.
  for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
    const RawBlock &raw_block = packet->blocks[blk_idx];

    for (size_t blk_fir_idx = 0; blk_fir_idx < FIRINGS_PER_BLOCK;
         ++blk_fir_idx) {
      size_t fir_idx = blk_idx * FIRINGS_PER_BLOCK + blk_fir_idx;

      double azimuth_diff = 0.0;
      if (fir_idx < FIRINGS_PER_PACKET - 1)
        azimuth_diff =
          firings[fir_idx + 1].firing_azimuth - firings[fir_idx].firing_azimuth;
      else
        azimuth_diff =
          firings[fir_idx].firing_azimuth - firings[fir_idx - 1].firing_azimuth;

      for (size_t scan_fir_idx = 0; scan_fir_idx < SCANS_PER_FIRING;
           ++scan_fir_idx) {
        size_t byte_idx =
          RAW_SCAN_SIZE * (SCANS_PER_FIRING * blk_fir_idx + scan_fir_idx);

        // Azimuth
        firings[fir_idx].azimuth[scan_fir_idx] =
          firings[fir_idx].firing_azimuth
          + (scan_fir_idx * DSR_TOFFSET / FIRING_TOFFSET) * azimuth_diff;

        // Distance
        TwoBytes raw_distance;
        raw_distance.bytes[0] = raw_block.data[byte_idx];
        raw_distance.bytes[1] = raw_block.data[byte_idx + 1];
        firings[fir_idx].distance[scan_fir_idx] =
          static_cast<double>(raw_distance.distance) * DISTANCE_RESOLUTION;

        // Intensity
        firings[fir_idx].intensity[scan_fir_idx] =
          static_cast<double>(raw_block.data[byte_idx + 2]);
      }
    }
  }

  return;
}

void N301_decoder::saveOneSweep()
{
  lidar->num_ranges = 0;
  lidar->azimuth.clear();
  lidar->ranges.clear();
  lidar->intensities.clear();
  lidar->x.clear();
  lidar->y.clear();
  lidar->num_ranges = sweep_data->scans[0].points.size();
  lidar->ranges.reserve(lidar->num_ranges);
  lidar->intensities.reserve(lidar->num_ranges);
  for (uint16_t i = 0; i < lidar->num_ranges; i++) {
    lidar->azimuth.push_back(sweep_data->scans[0].points[i].azimuth);
    lidar->ranges.push_back(sweep_data->scans[0].points[i].distance);
    lidar->intensities.push_back(sweep_data->scans[0].points[i].intensity);
    lidar->x.push_back(sweep_data->scans[0].points[i].x);
    lidar->y.push_back(sweep_data->scans[0].points[i].y);

    //         std::cout << sweep_data->scans[0].points[i].azimuth
    // 					<< "|" << sweep_data->scans[0].points[i].distance
    // 					<< "|" << sweep_data->scans[0].points[i].time << " " /*<<
    // std::endl*/;
  }

  *ready_data = *lidar;
  sweep_data_ready = true;
}
