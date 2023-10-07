#include "lowpass_filter.h"

LowPassFilter::LowPassFilter() : old_data{0, {0, 0, 0, 0, 0, 0, 0, 0, 0}}
{
}

m5::imu_data_t LowPassFilter::CalculateLowPassFilter(m5::imu_data_t new_data, float gyro_ratio, float accel_ratio)
{
  m5::imu_data_t res = new_data;

  m5::imu_3d_t old_gyro = old_data.gyro;
  m5::imu_3d_t new_gyro = new_data.gyro;
  res.gyro.x = old_gyro.x * gyro_ratio + new_gyro.x * (1 - gyro_ratio);
  res.gyro.y = old_gyro.y * gyro_ratio + new_gyro.y * (1 - gyro_ratio);
  res.gyro.z = old_gyro.z * gyro_ratio + new_gyro.z * (1 - gyro_ratio);

  m5::imu_3d_t old_accel = old_data.accel;
  m5::imu_3d_t new_accel = new_data.accel;
  res.accel.x = old_accel.x * accel_ratio + new_accel.x * (1 - accel_ratio);
  res.accel.y = old_accel.y * accel_ratio + new_accel.y * (1 - accel_ratio);
  res.accel.z = old_accel.z * accel_ratio + new_accel.z * (1 - accel_ratio);

  old_data = new_data;

  return res;
}
