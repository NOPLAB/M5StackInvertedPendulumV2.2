#ifndef M5STACK_KALMANFILTER_H
#define M5STACK_KALMANFILTER_H

#include <M5Unified.h>
#include "Kalman.h"
#include "robot_angle.h"

class KalmanFilter
{
private:
  Kalman kalmanX;
  Kalman kalmanY;
  float timer;

public:
  KalmanFilter();
  RobotAngle CalculateKalmanFilter(m5::imu_data_t data);
};

#endif // M5STACK_KALMANFILTER_H
