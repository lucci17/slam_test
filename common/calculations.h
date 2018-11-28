#pragma once

#include "macro_defines.h"
#include "struct_defines.h"
#include <cmath>

namespace common
{
namespace calculations
{

double NormalizeAngle(double angle)
{
  while (angle <= -M_PI)
    angle += DOUBLE_M_PI;
  while (angle > M_PI)
    angle -= DOUBLE_M_PI;

  return angle;
}

inline double calculate_euler_roll(Bgs::Quaternion &q)
{
  return atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
}

inline double calculate_euler_pitch(Bgs::Quaternion &q)
{
  return asin(2 * (q.w * q.y - q.z * q.x));
}

inline double calculate_euler_yaw(Bgs::Quaternion &q)
{
  return atan2(2 * (q.w * q.z + q.y * q.x), 1 - 2 * (q.y * q.y + q.z * q.z));
}

inline float distance_of_2_points(Bgs::Point &src, Bgs::Point &des)
{
  return (sqrt((src.x - des.x) * (src.x - des.x)
               + (src.y - des.y) * (src.y - des.y)
               + (src.z - des.z) * (src.z - des.z)));
}

inline void calculate_euler(Bgs::Quaternion &q)
{
  double euler_roll = calculate_euler_roll(q);
  double euler_pitch = calculate_euler_pitch(q);
  double euler_yaw = calculate_euler_yaw(q);
  PRINT_DEBUG_FMT("eular roll = %lf, pitch = %lf, yaw = %lf", euler_roll,
                  euler_pitch, euler_yaw);
}
} // namespace calculations
} // namespace common

// EOF calculations.h