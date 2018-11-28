#ifndef SYSTEM_STARTER_H
#define SYSTEM_STARTER_H

#include <cmath>
#include <error.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace system_starter
{

enum SystemState {
  kIdle,
  kMapping,
  kGatheringBaundry,
  kTransition,
  kStateCount
};

typedef struct {
  bool run_driver;
  bool run_carto;
  bool run_controller;

  std::string driver_config_filename;
  std::string carto_config_filename;
  std::string controller_config_filename;
} ProcessRunConfig;

typedef struct {
  bool all_process_run_background;
  int32_t server_port;
  SystemState start_state;
  std::string driver_process_path;
  std::string carto_process_path;
  std::string controller_process_path;

  ProcessRunConfig state_run_config[kStateCount];
} Config;

typedef struct {
  unsigned shm_init_succeed : 1;
  unsigned shm_sem_init_succeed : 1;
  unsigned carto_sem_init_succeed : 1;
  unsigned imu_sem_init_succeed : 1;
  unsigned gps_sem_init_succeed : 1;

  unsigned reserved : 27;
} Status;
};

int32_t init_singals();
int32_t create_and_init_shm();
void destroy_shm();
int32_t read_configs(int argc, char **argv, system_starter::Config &config);

int32_t create_driver_process(system_starter::Config &config);
int32_t create_carto_process(system_starter::Config &config);
int32_t create_controller_process(system_starter::Config &config);

// int32_t server_handle_msg(const std::string &message);
int32_t system_state_machine(system_starter::SystemState state);

#endif