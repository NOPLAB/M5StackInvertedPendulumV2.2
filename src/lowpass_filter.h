#ifndef M5STACK_LOWPASSFILTER_H
#define M5STACK_LOWPASSFILTER_H

#include <M5Unified.h>

class LowPassFilter {
private:
  m5::imu_data_t old_data;
public:
  LowPassFilter();
  m5::imu_data_t CalculateLowPassFilter(m5::imu_data_t new_data, float gyro_ratio, float accel_ratio);
};


#endif //M5STACK_LOWPASSFILTER_H
