#include "n301_driver.h"
#include <unistd.h>

int main()
{
  N301_driver driver;
  N301_lidar *lidar = new N301_lidar();

  while (true) {
    if (driver.get_sweep_data(lidar))
      printf(" data ready..., lidar range num = %lf \n", lidar->num_ranges);
  }

  return 0;
}