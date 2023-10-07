#include "kalman_filter.h"

KalmanFilter::KalmanFilter()
{
  timer = 0;
}

RobotAngle KalmanFilter::CalculateKalmanFilter(m5::imu_data_t data)
{
  float dt = ((float)micros() - timer) / 1000000;
  timer = (float)micros();
  RobotAngle res{};
  float accel_x = atan2(data.accel.y, data.accel.z) * static_cast<float>(RAD_TO_DEG);
  float accel_y = atan(-data.accel.x / sqrtf(data.accel.y * data.accel.y + data.accel.z * data.accel.z)) *
                  static_cast<float>(RAD_TO_DEG);
  res.x = kalmanX.getAngle(accel_x, data.gyro.x, dt);
  res.y = kalmanY.getAngle(accel_y, data.gyro.y, dt);
  return res;
}
