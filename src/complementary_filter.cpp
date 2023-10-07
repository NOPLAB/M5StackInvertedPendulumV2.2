#include "complementary_filter.h"

RobotAngle ComplementaryFilter::CalculateComplementaryFilter(m5::imu_data_t data, float coef)
{
  // ジャイロと加速度のそれぞれの角度の取得
  float gyro_x = data.gyro.x * (static_cast<float>(data.usec - pre_imu.usec) / (1000 * 1000)) + pre_angle.x;
  float gyro_y = data.gyro.y * (static_cast<float>(data.usec - pre_imu.usec) / (1000 * 1000)) + pre_angle.y;

  float accel_x = atan2(data.accel.y, data.accel.z) * static_cast<float>(RAD_TO_DEG);
  float accel_y = atan(-data.accel.x / sqrtf(data.accel.y * data.accel.y + data.accel.z * data.accel.z)) *
                  static_cast<float>(RAD_TO_DEG);

  RobotAngle res{0, 0, 0};

  // 相補
  res.x = gyro_x * (1 - coef) + accel_x * coef;
  res.y = gyro_y * (1 - coef) + accel_y * coef;

  pre_angle.x = res.x;
  pre_angle.y = res.y;

  pre_imu = data;

  return res;
}
