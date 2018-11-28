#include "LMS1xx.h"
#include "struct_defines.h"
#include <csignal>
#include <cstdio>
#include <string>

#define DEG2RAD M_PI / 180.0
using namespace Bgs;
using namespace LASER;

int main(int argc, char **argv)
{
  LMS1xx laser;
  scanCfg cfg;
  scanOutputRange outputRange;
  scanDataCfg dataCfg;

  std::string host = "192.168.0.1";
  std::string frame_id = "horizontal_laser_link";
  int port = 2111;
  LaserScanMsg scan_msg;

  while (true) {
    laser.connect(host, port);
    if (!laser.isConnected()) {
      sleep(1);
      continue;
    }

    PRINT_DEBUG("Logging in to laser.");
    laser.login();
    cfg = laser.getScanCfg();
    outputRange = laser.getScanOutputRange();

    if (cfg.scaningFrequency != 5000) {
      laser.disconnect();
      sleep(1);
      continue;
    }

    scan_msg.header.frame_id = frame_id;
    scan_msg.range_min = 0.01;
    scan_msg.range_max = 20.0;
    scan_msg.scan_time = 100.0 / cfg.scaningFrequency;

    std::cout << "scan_msg.angle range: " << outputRange.startAngle << " "
              << outputRange.stopAngle << std::endl;
    scan_msg.angle_increment =
      (double)outputRange.angleResolution / 10000.0 * DEG2RAD;
    scan_msg.angle_min =
      (double)outputRange.startAngle / 10000.0 * DEG2RAD - M_PI / 2;
    scan_msg.angle_max =
      (double)outputRange.stopAngle / 10000.0 * DEG2RAD - M_PI / 2;

    int angle_range = outputRange.stopAngle - outputRange.startAngle;
    int num_values = angle_range / outputRange.angleResolution;
    if (angle_range % outputRange.angleResolution == 0) {
      // Include endpoint
      ++num_values;
    }
    // scan_msg.ranges.resize(num_values);
    // scan_msg.intensities.resize(num_values);

    scan_msg.time_increment = (outputRange.angleResolution / 10000.0) / 360.
                              / (cfg.scaningFrequency / 100.0);

    dataCfg.outputChannel = 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.deviceName = false;
    dataCfg.outputInterval = 1;

    PRINT_DEBUG("Setting scan data configuration.");
    laser.setScanDataCfg(dataCfg);

    PRINT_DEBUG("Starting measurements.");
    laser.startMeas();

    status_t stat = laser.queryStatus();
    sleep(1);
    if (stat != ready_for_measurement) {
      PRINT_WARNING("Laser not ready. Retrying initialization.");
      laser.disconnect();
      sleep(1);
      continue;
    }

    PRINT_DEBUG("Starting device.");
    laser.startDevice(); // Log out to properly re-enable system after config

    PRINT_DEBUG("Commanding continuous measurements.");
    laser.scanContinous(1);

    while (1) {

      scan_msg.header.stamp = BgsTime::get_current_time();
      ++scan_msg.header.u_seq_num;

      scanData data;
      // 			PRINT_DEBUG("Reading scan data.");
      if (laser.getScanData(&data)) {
        // 				PRINT_DEBUG("Publishing scan data.");
        // 				for (int i = 0; i < data.dist_len1; i++)
        // 				{
        // 					scan_msg.ranges[i] = data.dist1[i] * 0.001;
        // 					printf("%f ", scan_msg.ranges[i]);
        // 				}
        // 				printf("\n");

        for (int i = 0; i < data.rssi_len1; i++) {
          scan_msg.intensities[i] = data.rssi1[i];
        }

        // PRINT_DEBUG("Publishing scan data.");
        // scan_pub.publish(scan_msg);
      } else {
        PRINT_ERROR(
          "Laser timed out on delivering scan, attempting to reinitialize.");
        break;
      }
    }

    laser.scanContinous(0);
    laser.stopMeas();
    laser.disconnect();
  }

  return 0;
}
