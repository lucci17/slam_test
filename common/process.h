/*!
 * @file process.h
 * @brief Head file for some common functions.
 *
 * - pidof used to get the process id (linux) of specific process
 * - is_single_process and reset_lock_file to make sure that
 * a process has been run only once.
 *
 * @author edward.liu
 * @date 2018-10-08
 */

#pragma once

#include <errno.h>
#include <error.h>
#include <stdio.h>
#include <string.h>
#include <sys/file.h>
#include <sys/types.h>
#include <unistd.h>

/*!
 * @brief namespace for structs and some common used functions
 *
 * - defined the data structs used in diffierent process, like BgsTime
 * - defined some functions that can be used in diffierent process
 * - some macros that can be used independently
 */
namespace common
{

/*!
 * @brief get the process id in linux system
 *
 * @param process_name like "driver" "carto" ...
 * @return the id, if there is no process
 * named process_name, then return -1
 */
int32_t pidof(const char *process_name)
{
  char cmd[128] = { 0 };
  sprintf(cmd, "ps -e | grep \'%s\' | awk \'{print $1}\'", process_name);
  char ret_buffer[32] = { 0 };
  FILE *file = popen(cmd, "r");
  int pid = -1;
  while (fgets(ret_buffer, 32, file) != NULL) {
    pid = atoi(ret_buffer);
  }
  pclose(file);

  return pid;
}

/*!
 * @brief make sure that a process only
 * have one instrance
 *
 * @param filename the locked filename, such as "driver.lck"
 * (we use the system call: lockf to get the process running status)
 * @param fd return the file description id of lockfile
 *
 * @return has the process already running?
 *
 * @note you should use the function
 * reset_lock_file to unlock the file you locked
 */
bool is_single_process(const char *filename, int32_t &fd)
{
  fd = open(filename, O_CREAT | O_RDWR, 0666);
  if (fd < 0) {
    perror("open file: ");
    return false;
  }
  int ret = lockf(fd, F_TLOCK, 0);
  if (ret < 0) {
    if (errno == EACCES || errno == EAGAIN)
      perror("file already locked: ");
    else
      perror("other error: ");

    return false;
  }

  return true;
}

/*!
 * @brief use this function to unlock the file
 * you locked in is_single_process function
 *
 * @param fd got from the function: is_single_process
 */
void reset_lock_file(const int32_t &fd)
{
  if (fd > 0) {
    close(fd);
    lockf(fd, F_ULOCK, 0);
  }
}

#define VMRSS_LINE 17
#define VMSIZE_LINE 13
#define PROCESS_ITEM 14

typedef struct {
  unsigned long user;
  unsigned long nice;
  unsigned long system;
  unsigned long idle;
} TotalCpuOccupy;

typedef struct {
  unsigned int pid;
  unsigned long utime;  // user time
  unsigned long stime;  // kernel time
  unsigned long cutime; // all user time
  unsigned long cstime; // all dead time
} ProcCpuOccupy;

const char *get_items(const char *buffer, int item)
{
  const char *p = buffer;
  if (item == 0)
    return p;

  int len = strlen(buffer);
  int count = 0;

  for (int i = 0; i < len; i++) {
    if (' ' == *p) {
      count++;
      if (count == item - 1) {
        p++;
        break;
      }
    }
    p++;
  }

  return p;
}

unsigned long get_cpu_total_occupy()
{
  FILE *fd;
  char buff[1024] = { 0 };
  TotalCpuOccupy t;

  fd = fopen("/proc/stat", "r");
  if (NULL == fd) {
    return 0;
  }

  fgets(buff, sizeof(buff), fd);
  char name[64] = { 0 };
  sscanf(buff, "%s %ld %ld %ld %ld", name, &t.user, &t.nice, &t.system,
         &t.idle);
  fclose(fd);

  return (t.user + t.nice + t.system + t.idle);
}

unsigned long get_cpu_proc_occupy(unsigned int pid)
{
  char file_name[64] = { 0 };
  ProcCpuOccupy t;
  FILE *fd;
  char line_buff[1024] = { 0 };
  sprintf(file_name, "/proc/%d/stat", pid);

  fd = fopen(file_name, "r");
  if (nullptr == fd) {
    return 0;
  }

  fgets(line_buff, sizeof(line_buff), fd);

  sscanf(line_buff, "%u", &t.pid);
  const char *q = get_items(line_buff, PROCESS_ITEM);
  sscanf(q, "%ld %ld %ld %ld", &t.utime, &t.stime, &t.cutime, &t.cstime);
  fclose(fd);

  return (t.utime + t.stime + t.cutime + t.cstime);
}

/*!
 * \brief returns the cpu occupancy rate of 'pid' (process id)
 *
 * \param pid process id in linux system
 *
 * \return cpu occupy, unit: %
 */
float get_proc_cpu(uint32_t pid)
{
  unsigned long totalcputime1, totalcputime2;
  unsigned long procputime1, procputime2;

  totalcputime1 = get_cpu_total_occupy();
  procputime1 = get_cpu_proc_occupy(pid);

  usleep(400000);

  totalcputime2 = get_cpu_total_occupy();
  procputime2 = get_cpu_proc_occupy(pid);

  float pcpu = 0.0f;
  if (0 != totalcputime2 - totalcputime1) {
    pcpu =
      100.0 * (procputime2 - procputime1) / (totalcputime2 - totalcputime1);
  }
  if (pcpu < 0.f)
    pcpu = 0.f;

  return pcpu;
}

} // namespace common
