#include <Arduino.h>
#include <VL53L1X_ULD.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MCP23X08.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <config.h>
#include <sensors.h>
#include <motors.h>
#include <start_module.h>
#include <tuner.h>

Adafruit_NeoPixel pixels(7, leds, NEO_GRB + NEO_KHZ800);
Adafruit_MCP23X08 mcp;
Adafruit_MPU6050 mpu;
VL53L1X_ULD sensor[sc];
VL53L1X_Result_t results[sc];
Preferences prefs_global;

float threshold = 500.0f;

float Kp = 75.0f;
float Kd = 4.0f;
float slowKp1 = 15.0f;
float slowKd1 = 0.5f;
float slowKp2 = 15.0f;
float slowKd2 = 0.5f;
float gyroKp = 2.4f;
float gyroKd = 0.07f;

int mode_count = 4;
int mode = 1;
int dyn_mode = 1;

float left_speed, right_speed, ramp_up1, ramp_up2, gyro_output, to_target, bias, error, output, target_angle, target_yaw;
float yaw = 0.0f;

float base_speed = 50.0f;
float spin_speed = 50.0f;
float slow_speed = 30.0f;
float ramp_up_step = 0.75f;
int last_dir = -1;
bool slow_down = false;

bool en_gyro = true;
bool callibrate_flag = false;
bool target_reached = false;

void callibrate_gyro()
{
  pixels.clear();
  callibrate_flag = false;
  hold_led = true;
  yaw = 0.0f;
  int samples = 200;
  float sum = 0;
  sensors_event_t a, g, temp;
  for (int i = 0; i < samples; i++)
  {
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;

    int led = map(i, 0, samples, 0, leds);

    pixels.setPixelColor(round(led), pixels.Color(15, 0, 15));
    pixels.show();
  }
  bias = sum / (float)samples;

  delay(400);
  hold_led = false;
  pixels.clear();
  pixels.show();

  // Serial.println("Gyro callibrated, bias: " + String(bias));

  prefs_global.begin("robot", false);
  prefs_global.putFloat("bias", bias);
  prefs_global.end();
}

void handle_mode()
{
  static unsigned long hold_time = 0;

  bool pressed = false;
  bool held = false;

  if (!digitalRead(btn))
  {
    hold_time = millis() + 700;
    while (!digitalRead(btn))
    {
      if (millis() > hold_time)
      {
        held = true;
        break;
      }
    }
    if (!held)
      pressed = true;
  }

  if (pressed)
  {
    hold_led = true;

    mode++;
    mode = (mode > mode_count) ? 1 : mode;
    dyn_mode = mode;

    pixels.clear();
    for (int i = 0; i < dyn_mode; i++)
      pixels.setPixelColor(i, pixels.Color(30, 30, 0));
    pixels.show();

    prefs_global.begin("robot", false);
    prefs_global.putInt("start_mode", mode);
    prefs_global.end();

    if (dyn_mode == 1)
      change_settings(def);
    else if (dyn_mode == 2)
      change_settings(def);
    else if (dyn_mode == 3)
      change_settings(def);
    else if (dyn_mode == 4)
      change_settings(def);
    else if (dyn_mode == 5)
      change_settings(def);

    while (!digitalRead(btn))
      delay(10);

    delay(400);
    hold_led = false;
    pixels.clear();
    pixels.show();
  }
  else if (held)
  {
    hold_led = true;
    callibrate_flag = true;
    pixels.clear();
    pixels.fill(pixels.Color(15, 0, 15));
    pixels.show();

    while (!digitalRead(btn))
      delay(10);

    pixels.clear();
    pixels.show();

    delay(400);
    hold_led = false;
  }
}

void setup()
{
  Serial.begin(115200);
  pixels.begin();
  Wire.begin(sda, scl, 500000);

  pinMode(btn, INPUT_PULLUP);

  setup_motors();
  setup_sensors();
  startIRTask((uint8_t)rcv);

  mpu.begin(0x68, &Wire);
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  mpu.setSampleRateDivisor(0);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);

  prefs_global.begin("robot", false);
  Kp = prefs_global.getFloat("kp", 75.0);
  Kd = prefs_global.getFloat("kd", 4.0);
  slowKp1 = prefs_global.getFloat("skp1", 15.0);
  slowKd1 = prefs_global.getFloat("skd1", 0.5);
  slowKp2 = prefs_global.getFloat("skp2", 15.0);
  slowKd2 = prefs_global.getFloat("skd2", 0.5);
  gyroKp = prefs_global.getFloat("gkp", 2.4);
  gyroKd = prefs_global.getFloat("gkd", 0.07);
  base_speed = prefs_global.getFloat("bs", 50.0);
  spin_speed = prefs_global.getFloat("sp", 50.0);
  target_angle = prefs_global.getFloat("targ", 0.0);
  bias = prefs_global.getFloat("bias", 0.0);
  dyn_mode = mode = prefs_global.getInt("start_mode", 1);
  threshold = prefs_global.getFloat("threshold", 500);
  prefs_global.end();

  pixels.clear();
  for (int i = 0; i < dyn_mode; i++)
    pixels.setPixelColor(i, pixels.Color(30, 30, 0));
  pixels.show();

  delay(400);

  pixels.clear();
  pixels.show();

  static TuningParam mySettings[] = {
      {"PID Error", "err", &error, 0, 0, 0, TYPE_READONLY},
      {"PID Output", "out", &output, 0, 0, 0, TYPE_READONLY},
      {"Threshold", "threshold", &threshold, 0, 1000, 5, TYPE_ARROWS},
      {"Gyro Position", "gpos", &yaw, 0, 0, 0, TYPE_READONLY},
      {"Gyro Output", "gout", &gyro_output, 0, 0, 0, TYPE_READONLY},
      {"Reset yaw", "reset_yaw", &yaw, 0, 0, 0, TYPE_BUTTON},
      {"Drive Kp", "kp", &Kp, 0, 100, 0.5, TYPE_ARROWS},
      {"Drive Kd", "kd", &Kd, 0, 100, 0.5, TYPE_ARROWS},
      {"Base Speed", "bs", &base_speed, 0, 100, 1, TYPE_ARROWS},
      {"Spin Speed", "sp", &spin_speed, 0, 100, 1, TYPE_ARROWS},
      {"Slow Kp1", "skp1", &slowKp1, 0, 100, 0.5, TYPE_ARROWS},
      {"Slow Kd1", "skd1", &slowKd1, 0, 100, 0.5, TYPE_ARROWS},
      {"Slow Kp2", "skp2", &slowKp2, 0, 100, 0.5, TYPE_ARROWS},
      {"Slow Kd2", "skd2", &slowKd2, 0, 100, 0.5, TYPE_ARROWS},
      {"Gyro Kp", "gkp", &gyroKp, 0, 10, 0.05, TYPE_ARROWS},
      {"Gyro Kd", "gkd", &gyroKd, 0, 10, 0.01, TYPE_ARROWS},
      {"Target Angle", "targ", &target_angle, -180, 180, 5, TYPE_SLIDER},

  };
  startTuner(mySettings, sizeof(mySettings) / sizeof(mySettings[0]), results, &prefs_global);
}

struct PIDState
{
  float prev_error = 0;
  float prev_derivative = 0;
  float integral = 0;
};

PIDState drivePID, gyroPID;

float pid(float error, float dt, float Kp, float Ki, float Kd, PIDState &state, float alpha = 1.0f, float integral_limit = 1000.0f)
{
  if (dt < 0.002f)
    dt = 0.002f;

  float P = Kp * error;

  state.integral += error * dt;
  state.integral = constrain(state.integral, -integral_limit, integral_limit);
  float I = Ki * state.integral;
  float raw_derivative = (error - state.prev_error) / dt;
  float derivative = alpha * raw_derivative + (1.0f - alpha) * state.prev_derivative;

  state.prev_error = error;
  state.prev_derivative = derivative;

  return P + I + Kd * derivative;
}

float dt;
unsigned long lastTime = 0;
unsigned long closeTime = 0;
unsigned long targetTime = 0;
unsigned long panicTime = 0;

void loop()
{
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  bool ut[sc];

  if (callibrate_flag)
    callibrate_gyro();

  if (en_gyro)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float rate = (g.gyro.z - bias) * RAD_TO_DEG;
    yaw += rate * dt;
    yaw = fmod(yaw + 360.0f, 360.0f);
    to_target = fmod((target_yaw - yaw) + 540.0f, 360.0f) - 180.0f;
    gyro_output = pid(to_target, dt, gyroKp, 0.0f, gyroKd, gyroPID, 1.0f);

    if (abs(to_target) <= 5.0f)
    {
      if (millis() - targetTime >= 20)
        target_reached = true;
    }
    else if (!target_reached)
    {
      targetTime = millis();
      target_reached = false;
    }

    if (dyn_mode == 5)
      target_reached = false;
  }

  en_gyro = (dyn_mode == 3 || dyn_mode == 4 || dyn_mode == 5) && !target_reached;

  if (!en_gyro || !started)
    read_sensors(results, &error, ut);

  if ((!started) ? results[0].Distance < 100 || results[1].Distance < 100 || results[2].Distance < 100 : error < -0.01)
    last_dir = -1;
  else if ((!started) ? results[4].Distance < 100 || results[5].Distance < 100 || results[6].Distance < 100 : error > 0.01)
    last_dir = 1;

  if (slow_down)
    output = pid(error, dt, slowKp1, 0.0f, slowKd1, drivePID, 0.75f);
  else if (dyn_mode == 2 || dyn_mode == 3)
    output = pid(error, dt, slowKp2, 0.0f, slowKd2, drivePID, 0.5f);
  else
    output = pid(error, dt, Kp, 0.0f, Kd, drivePID, 0.75f);

  bool any_ut1 = false;
  bool any_ut2 = false;
  for (int i = 0; i < sc; i++)
  {
    if (ut[i])
    {
      any_ut1 = true;
      break;
    }
    if (results[i].Distance < threshold / 2)
    {
      any_ut2 = true;
      break;
    }
  }

  digitalWrite(stby, started);

  if (started)
  {
    // MODE 1
    if (dyn_mode == 1)
    {
      if (results[2].Distance < 100 || results[3].Distance < 100 || results[4].Distance < 100)
      {

        if (closeTime == 0)
          closeTime = now;

        slow_down = (now - closeTime < 500);
      }
      else
      {
        slow_down = false;
        closeTime = 0;
      }

      if (any_ut1)
      {

        if (ut[2] || ut[3] || ut[4])
          ramp_up1 = constrain(ramp_up1 + ramp_up_step, 0, 100);

        if (slow_down)
          ramp_up1 = slow_speed - base_speed;

        left_speed = base_speed + ramp_up1 + output;
        right_speed = base_speed + ramp_up1 - output;

        if (ut[0] || ut[6])
        {
          left_speed = spin_speed * last_dir;
          right_speed = -spin_speed * last_dir;
        }
      }
      else
      {
        ramp_up1 = 0;
        left_speed = spin_speed * last_dir;
        right_speed = -spin_speed * last_dir;
      }
    }

    // MODE 2
    else if (dyn_mode == 2)
    {
      if (any_ut1)
      {
        left_speed = slow_speed + output;
        right_speed = slow_speed - output;
        if (results[3].Distance > 60)
        {
          closeTime = now;
        }
        else if (now - closeTime > 330)
        {
          dyn_mode = 1;
          ramp_up1 = slow_speed - base_speed;
        }

        if (ut[0] || ut[6])
        {
          left_speed = spin_speed * last_dir;
          right_speed = -spin_speed * last_dir;
        }
      }
      else
      {
        left_speed = spin_speed * last_dir;
        right_speed = -spin_speed * last_dir;
      }
    }
    // MODE 3
    else if (dyn_mode == 3)
    {
      left_speed = -gyro_output;
      right_speed = gyro_output;
      if (target_reached)
      {
        left_speed = slow_speed + output;
        right_speed = slow_speed - output;

        if (any_ut2 || now - panicTime > 1300)
          dyn_mode = 2;
      }
      else
      {
        panicTime = now;
      }
    }
    // MODE 4
    else if (dyn_mode == 4)
    {
      left_speed = slow_speed - gyro_output;
      right_speed = slow_speed + gyro_output;

      if (target_reached)
      {
        ramp_up2 = constrain(ramp_up2 + 8.0f, 0, 100);
        left_speed = slow_speed + ramp_up2;
        right_speed = slow_speed + ramp_up2;

        if (any_ut1 || now - panicTime > 300)
          dyn_mode = 1;
      }
      else
      {
        panicTime = now;
      }
    }
    // MODE 4 - GYRO DEMO
    else if (dyn_mode == 5)
    {
      left_speed = -gyro_output;
      right_speed = gyro_output;
    }

    drive(left_speed, right_speed);
  }
  else
  {
    handle_mode();

    panicTime = now;

    left_speed = 0;
    right_speed = 0;
    ramp_up1 = 0;
    ramp_up2 = 0;
    closeTime = 0;
    yaw = 0;

    slow_down = false;
    target_reached = false;

    target_yaw = -last_dir * target_angle;
  }

  // Serial.println("slow_down: " + String(slow_down));

  unsigned long loopTime = millis() - now;

  // Serial.printf("Yaw: %.2f\n", yaw);
  // printf("%lu\t(%d)%d\t(%d)%d\t(%d)%d\t(%d)%d\t(%d)%d\t(%d)%d\t(%d)%d\tmode: %d\terror: %.2f\toutput: %.2f\tlast_dir: %d\n\r", loopTime, results[0].Status, results[0].Distance, results[1].Status, results[1].Distance, results[2].Status, results[2].Distance, results[3].Status, results[3].Distance, results[4].Status, results[4].Distance, results[5].Status, results[5].Distance, results[6].Status, results[6].Distance, dyn_mode, error, output, last_dir);
}