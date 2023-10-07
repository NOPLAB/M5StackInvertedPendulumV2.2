#ifndef M5STACK_ROBOTIMU_H
#define M5STACK_ROBOTIMU_H

#include <M5Unified.h>
#include <Arduino.h>

class RobotImu {
private:
  bool is_calibrating;

  uint32_t calib_countdown_sec;

  uint32_t prev_sec;

  // Strength of the calibration operation;
  // 0: disables calibration.
  // 1 is weakest and 255 is strongest.
  static constexpr uint8_t calib_value = 64;

  m5::imu_data_t imu_data;

  void UpdateCalibration();

public:
  RobotImu();
  bool Update();
  void StartCalibration(uint32_t countdown_sec);
  m5::imu_data_t GetImuData();
  bool IsCalibrating();
};


#endif //M5STACK_ROBOTIMU_H
