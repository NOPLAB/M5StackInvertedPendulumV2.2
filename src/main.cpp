#include <M5Unified.h>
#include <Arduino.h>

#include "Wire.h"

#include "WiFi.h"
#include "ArduinoJson.h"

#include <ArduPID.h>

#include "robot_imu.h"
#include "lowpass_filter.h"
#include "complementary_filter.h"
// #include "kalman_filter.h"

#include "secret.h"

// NOPの顔が正面で上が0x08 下が0x09
// Slave ID #8 #9
const byte slave_addr_r = 0x08;
const byte slave_addr_l = 0x09;

RobotImu imu;

LowPassFilter low_pass_filter;
ComplementaryFilter c_filter;
// KalmanFilter kalman_filter;

RobotAngle angle;
constexpr int32_t angle_green = 5;

bool is_output = false;

uint32_t pid_T = 5;    // 5ms => 200Hz cycle frequency
uint32_t motor_T = 10; // 5ms => 200Hz cycle frequency
uint32_t motor_last_time = 0;

double angle_reference = -90;
double ap = 0.6;
double ai = 4;
double ad = 0.0006;
double a_in = 0;
double a_out = 0;
ArduPID angle_pid;

double position_reference = 0;
double pp = 0.006;
double pi = 0;
double pd = 0.00001;
double p_in = 0;
double p_out = 0;
ArduPID position_pid;

double motor_output = 0;
double position = 0;

constexpr double max_motor_output = 18;
constexpr uint32_t max_over_motor_output_time = 1000;
uint32_t over_motor_output_timer = 0;

constexpr uint32_t PORT = 65000;
WiFiServer server(PORT);

bool wifi_connected = false;

SemaphoreHandle_t xBinarySemaphore;

// uint32_t loop_time;

bool WifiConnect(const char *ssid, const char *pass)
{
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    M5.update();
    if (M5.BtnA.isPressed())
    {
      M5_LOGI("Cancel");
      delay(2000);
      return false;
    }
    M5_LOGI("Not Connecting...");
    delay(500);
  }

  M5_LOGI("Connected");
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextColor(WHITE);
  M5.Display.setCursor(0, 0);
  M5.Display.printf("%s", WiFi.localIP().toString().c_str());
  delay(3000);

  return true;
}

void send_motor_power(byte slaveADR, int16_t data)
{
  // Serial.println("send_motor_power:" + String(slaveADR));

  // intをbyteのHとLに分ける
  byte send_data_h = byte(data >> 8);
  byte send_data_l = byte(data << 8 >> 8);

  // 分割したデータ
  // Serial.println("sendData:" + String(send_data_h) + ":" + String(send_data_l));
  // Serial.println("sendData:" + String(int((send_data_h << 8) | send_data_l)));

  // 送信
  Wire.beginTransmission(slaveADR);
  Wire.write(send_data_h);
  Wire.write(send_data_l);
  Wire.endTransmission();

  // 送信完了
  // Serial.println("sended:");

  // 受信したデータを返すようにリクエスト
  /*Wire.requestFrom(slaveADR, 2);
  byte b = Wire.read();
  Serial.print("i2c:" + String(b));
  b = Wire.read();
  Serial.println(":" + String(b));

  // 受信完了
  Serial.println("received:");*/

  // Serial.println("----------");
}

int16_t hz_to_ms(double hz)
{
  return (int16_t)(1000 / hz);
}

void tcp_task(void *pvParameters)
{
  for (;;)
  {
    // クライアントを取得する
    WiFiClient client = server.available();

    // クライアントがいる
    if (client)
    {
      if (client.connected())
      {
        vTaskDelay(10);
        int size = client.available();
        if (size)
        {
          String str = "";
          str = client.readString();
          Serial.println(str);

          DynamicJsonDocument doc(1024);
          deserializeJson(doc, str);

          if (!doc["ap"].isNull())
          {
            ap = static_cast<float>(doc["ap"]);
          }
          if (!doc["ai"].isNull())
          {
            ai = static_cast<float>(doc["ai"]);
          }
          if (!doc["ad"].isNull())
          {
            ad = static_cast<float>(doc["ad"]);
          }
          if (!doc["pp"].isNull())
          {
            pp = static_cast<float>(doc["pp"]);
          }
          if (!doc["pi"].isNull())
          {
            pi = static_cast<float>(doc["pi"]);
          }
          if (!doc["pd"].isNull())
          {
            pd = static_cast<float>(doc["pd"]);
          }

          if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE)
          {
            angle_pid.setCoefficients(ap, ai, ad);
            position_pid.setCoefficients(pp, pi, pd);

            DynamicJsonDocument write_doc(1024);
            write_doc["ap"] = ap;
            write_doc["ai"] = ai;
            write_doc["ad"] = ad;
            write_doc["pp"] = pp;
            write_doc["pi"] = pi;
            write_doc["pd"] = pd;
            String write_str;
            serializeJson(write_doc, write_str);

            client.write(write_str.c_str());

            xSemaphoreGive(xBinarySemaphore);
          }
          else
          {
            Serial.println("SemaphoreTake failed.");
          }
        }
      }
      client.stop();
    }

    delay(10);
  }
}

void reset()
{
  send_motor_power(slave_addr_r, hz_to_ms(0));
  send_motor_power(slave_addr_l, hz_to_ms(0));

  if (xSemaphoreTake(xBinarySemaphore, (TickType_t)2000) == pdTRUE)
  {
    position = 0;
    motor_output = 0;

    angle_pid.reset();
    position_pid.reset();
    angle_pid.start();
    position_pid.start();

    xSemaphoreGive(xBinarySemaphore);
  }
}

void pid_compute()
{
  if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE)
  {
    //    double angle_error = (angle_reference - angle.x) / 90;
    a_in = angle.x;
    angle_pid.compute();

    //    double position_error = position_reference + position;
    p_in = position;
    position_pid.compute();

    motor_output = a_out + p_out;
    position += motor_output;

    // モーターの出力値のオーバー検出
    if (abs(motor_output) > max_motor_output)
    {
      motor_output = max_motor_output;

      // オーバーしている期間が設定値を超えるとリセットされる
      if (over_motor_output_timer > 0 && (millis() - over_motor_output_timer) > max_over_motor_output_time)
      {
        is_output = !is_output;
        over_motor_output_timer = 0;
        reset();
      }
      else if (over_motor_output_timer == 0)
      {
        over_motor_output_timer = millis();
      }
    }
    else
    {
      over_motor_output_timer = 0;
    }

    //    angle_pid.debug(&Serial, "angle", // Can include or comment out any of these terms to print
    //                    PRINT_OUTPUT | // in the Serial plotter
    //                    PRINT_BIAS |
    //                    PRINT_P |
    //                    PRINT_I |
    //                    PRINT_D);

    xSemaphoreGive(xBinarySemaphore);
  }
}

void setup()
{
  auto cfg = M5.config();

  M5.begin(cfg);

  M5.Log.setLogLevel(m5::log_target_serial, esp_log_level_t::ESP_LOG_VERBOSE);

  M5.Display.setRotation(3);

  M5_LOGI("M5 Begin");

  imu = *new RobotImu();

  angle_pid.begin(&a_in, &a_out, &angle_reference, ap, ai, ad);
  angle_pid.setSampleTime(pid_T);
  angle_pid.setOutputLimits(-20, 20);
  angle_pid.setBias(0);
  angle_pid.setWindUpLimits(-10, 10);

  position_pid.begin(&p_in, &p_out, &position_reference, pp, pi, pd);
  position_pid.setSampleTime(pid_T);
  position_pid.reverse();
  position_pid.setOutputLimits(-20, 20);
  position_pid.setBias(0);
  position_pid.setWindUpLimits(-10, 10);

  angle_pid.start();
  position_pid.start();

  xBinarySemaphore = xSemaphoreCreateMutex();

  if (WifiConnect(wifi_ssid, wifi_pass))
  {
    wifi_connected = true;
    server.begin();
    xTaskCreateUniversal(
        tcp_task,
        "tcp_task",
        8192,
        NULL,
        0,
        NULL,
        PRO_CPU_NUM);
  }
}

void loop()
{
  //  loop_time = millis();

  // IMUの値が更新されていたら
  if (imu.Update())
  {
    // IMUの値を取得する
    auto imu_data = imu.GetImuData();
    // ローパスフィルタにかける
    auto filter_res = low_pass_filter.CalculateLowPassFilter(imu_data, 0.35, 0.5);
    // 相補フィルタで値を取得する
    angle = c_filter.CalculateComplementaryFilter(filter_res, 0.03);
    // カルマンフィルタで値を取得する
    //    angle = kalman_filter.CalculateKalmanFilter(filter_res);

    //    Serial.printf("%f\n", angle.x);
    //    Serial.printf("%f, %f\n", angle.x, filter_res.gyro.x);

    // キャリブレート中でなければ
    if (!imu.IsCalibrating())
    {
      //      M5.Display.fillScreen(BLACK);
      //
      //      if (angle.x > angle_reference - angle_green && angle.x < angle_reference + angle_green) {
      //        M5.Display.setColor(GREEN);
      //      } else {
      //        M5.Display.setColor(WHITE);
      //      }
      //
      //      M5.Display.drawLine(M5.Display.width() / 2,
      //                          M5.Display.height(),
      //                          M5.Display.width() / 2 + cos((angle.x) * DEG_TO_RAD) * 100,
      //                          M5.Display.height() + sin((angle.x) * DEG_TO_RAD) * 100);
    }
  }

  // M5関係の更新
  M5.update();

  uint32_t now = millis();

  if (is_output)
  {
    pid_compute();
  }

  if ((now - motor_last_time) >= motor_T)
  {
    if (is_output)
    {
      motor_last_time = now;
      send_motor_power(slave_addr_r, hz_to_ms(motor_output));
      send_motor_power(slave_addr_l, hz_to_ms(-motor_output));
    }
  }

  // A + C Calibration
  if (M5.BtnA.isPressed() && !M5.BtnB.isPressed() && M5.BtnC.isPressed())
  {
    is_output = false;

    reset();

    imu.StartCalibration(5);
  }

  // A Toggle Motor Power
  if (M5.BtnA.wasClicked() && !M5.BtnB.isPressed() && !M5.BtnC.isPressed())
  {
    is_output = !is_output;

    reset();
  }

  // C Target Angle
  if (!M5.BtnA.isPressed() && M5.BtnB.wasClicked() && !M5.BtnC.isPressed())
  {
    angle_reference = angle.x;
  }

  delay(1);
  //  Serial.printf("%lu\n", millis() - loop_time);
}
