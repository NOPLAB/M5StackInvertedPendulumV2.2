#include "robot_imu.h"

RobotImu::RobotImu() : imu_data{0, {0, 0, 0, 0, 0, 0, 0, 0, 0}}
{
  is_calibrating = false;
  calib_countdown_sec = 0;
  prev_sec = 0;

  const char *name;
  switch (M5.Imu.getType())
  {
  case m5::imu_none:
    name = "not found";
    break;
  case m5::imu_sh200q:
    name = "sh200q";
    break;
  case m5::imu_mpu6050:
    name = "mpu6050";
    break;
  case m5::imu_mpu6886:
    name = "mpu6886";
    break;
  case m5::imu_mpu9250:
    name = "mpu9250";
    break;
  case m5::imu_bmi270:
    name = "bmi270";
    break;
  default:
    name = "unknown";
    break;
  };
  M5_LOGI("RobotImu:%s", name);

  // NVS(不揮発メモリ)からオフセットを読み込めなかった
  if (!M5.Imu.loadOffsetFromNVS())
  {
    StartCalibration(10);
  }
}

bool RobotImu::Update()
{
  auto imu_update = M5.Imu.update();

  uint32_t sec = millis() / 1000;
  if (prev_sec != sec)
  {
    prev_sec = sec;

    if (is_calibrating)
    {
      calib_countdown_sec--;
      UpdateCalibration();
    }
  }

  if (imu_update)
  {
    imu_data = M5.Imu.getImuData();

    return true;
  }
  else
  {
    return false;
  }
}

void RobotImu::StartCalibration(uint32_t countdown_sec)
{
  calib_countdown_sec = countdown_sec;
  is_calibrating = true;
  UpdateCalibration();
}

void RobotImu::UpdateCalibration()
{
  if (calib_countdown_sec > 0)
  { // Start Calibration
    is_calibrating = true;
    M5.Imu.setCalibration(calib_value, calib_value, calib_value);
    // ※ The actual calibration operation is performed each time during M5.imu.update.
  }
  else
  { // Stop Calibration
    is_calibrating = false;
    M5.Imu.setCalibration(0, 0, 0);

    M5.Imu.saveOffsetToNVS();
  }

  M5.Display.fillScreen(BLACK);

  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(10);

  M5.Display.setCursor(0, 0);
  M5.Display.print("Countdown:");

  M5.Display.setCursor(100, 155);
  M5.Display.printf("%d", calib_countdown_sec);
}

m5::imu_data_t RobotImu::GetImuData()
{
  return imu_data;
}

bool RobotImu::IsCalibrating()
{
  return is_calibrating;
}
