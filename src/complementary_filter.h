#ifndef M5STACK_COMPLEMENTARYFILTER_H
#define M5STACK_COMPLEMENTARYFILTER_H

#include <M5Unified.h>
#include "robot_angle.h"

class ComplementaryFilter
{
private:
  RobotAngle pre_angle;
  m5::imu_data_t pre_imu;

public:
  RobotAngle CalculateComplementaryFilter(m5::imu_data_t data, float coef);
};

#endif // M5STACK_COMPLEMENTARYFILTER_H
