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
VL53L1X_ULD sensor[sc];

Preferences prefs_global;

float Kp = 50.0;
float Kd = 12.0;

float gyroKp = 1.0;
float gyroKd = 1.0;

int mode = 1;
int dyn_mode = 1;

float left_speed, right_speed, ramp_up1, gyro_output, to_target, bias;

float base_speed = 67;
float spin_speed = 50;
int last_dir = -1;

float yaw = 0;
float target_angle = 0.0f;
float en_gyro = true;
float callibrate_flag = false;

float error = 0;
float output = 0;
VL53L1X_Result_t results[sc];
Adafruit_MPU6050 mpu;

void callibrate_gyro()
{
  int samples = 200;
  float sum = 0;
  sensors_event_t a, g, temp;
  for (int i = 0; i < samples; i++)
  {
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
  }
  bias = sum / (float)samples;

  prefs_global.begin("robot", false);
  prefs_global.putFloat("bias", bias);
  prefs_global.end();
}

void handle_mode()
{

  static unsigned long hold_time = 0;
  bool pressed = false;

  if (digitalRead(btn))
    hold_time = millis();

  pressed = (millis() - hold_time > 150);

  if (pressed && !started)
  {
    hold_led = true;

    mode++;
    mode = (mode > 3) ? 1 : mode;
    dyn_mode = mode;

    pixels.clear();
    for (int i = 0; i < dyn_mode; i++)
      pixels.setPixelColor(i, pixels.Color(75, 50, 0));
    pixels.show();

    while (!digitalRead(btn))
      delay(10);

    prefs_global.begin("robot", false);
    prefs_global.putInt("start_mode", mode);
    prefs_global.end();

    if (dyn_mode == 1)
      change_settings(def);
    if (dyn_mode == 2)
      change_settings(medium);

    delay(400);
    hold_led = false;
    pixels.clear();
    pixels.show();
  }
}

void load_mode()
{
  prefs_global.begin("robot", true);
  mode = prefs_global.getInt("start_mode", 1);
  prefs_global.end();
  dyn_mode = mode;

  pixels.clear();
  for (int i = 0; i < dyn_mode; i++)
    pixels.setPixelColor(i, pixels.Color(75, 50, 0));
  pixels.show();

  delay(400);

  pixels.clear();
  pixels.show();
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

  load_mode();

  prefs_global.begin("robot", false);
  Kp = prefs_global.getFloat("kp", 150.0);
  Kd = prefs_global.getFloat("kd", 50.0);
  gyroKp = prefs_global.getFloat("gkp", 1.0);
  gyroKd = prefs_global.getFloat("gkd", 1.0);
  base_speed = prefs_global.getFloat("bs", 30.0);
  spin_speed = prefs_global.getFloat("sp", 50.0);
  target_angle = prefs_global.getFloat("targ", 180.0);
  bias = prefs_global.getFloat("gx_bias", 0.0);
  prefs_global.end();

  static TuningParam mySettings[] = {
      {"PID Error", "err", &error, 0, 0, 0, TYPE_READONLY},
      {"PID Output", "out", &output, 0, 0, 0, TYPE_READONLY},
      {"Gyro Position", "gpos", &yaw, 0, 0, 0, TYPE_READONLY},
      {"Gyro Output", "gout", &gyro_output, 0, 0, 0, TYPE_READONLY},
      {"Reset yaw", "reset_yaw", &yaw, 0, 0, 0, TYPE_BUTTON},
      {"Callibrate", "callibrate", &callibrate_flag, 1, 0, 0, TYPE_BUTTON},
      {"Drive Kp", "kp", &Kp, 0, 100, 1, TYPE_ARROWS},
      {"Drive Kd", "kd", &Kd, 0, 100, 0.5, TYPE_ARROWS},
      {"Base Speed", "bs", &base_speed, 0, 100, 1, TYPE_ARROWS},
      {"Spin Speed", "sp", &spin_speed, 0, 100, 1, TYPE_ARROWS},
      {"Gyro Kp", "gkp", &gyroKp, 0, 10, 0.05, TYPE_ARROWS},
      {"Gyro Kd", "gkd", &gyroKd, 0, 10, 0.01, TYPE_ARROWS},
      {"Target Angle", "targ", &target_angle, -180, 180, 5, TYPE_SLIDER},

  };
  startTuner(mySettings, 13, results, &prefs_global);
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
unsigned long targetingTime = 0;

void loop()
{
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  handle_mode();

  bool ut[sc];

  if (callibrate_flag)
  {
    callibrate_gyro();
    callibrate_flag = false;
  }

  if (en_gyro)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float rate = (g.gyro.z - bias) * RAD_TO_DEG;
    yaw += rate * dt;
    yaw = fmod(yaw + 360.0f, 360.0f);
    to_target = fmod((target_angle - yaw) + 540.0f, 360.0f) - 180.0f;
    gyro_output = pid(to_target, dt, gyroKp, 0.0f, gyroKd, gyroPID, 1.0f);
  }
  else
  {
    read_sensors(results, &error, ut);
  }

  bool angle_reached = abs(to_target) < 5.0f;

  if (error < -0.01)
    last_dir = -1;
  else if (error > 0.1)
    last_dir = 1;

  output = pid(error, dt, Kp, 0.0f, Kd, drivePID, 1.0f);

  bool any_ut1 = false;
  for (int i = 0; i < sc; i++)
  {
    if (ut[i])
    {
      any_ut1 = true;
      break;
    }
  }

  digitalWrite(stby, started);

  if (started)
  {
    // MODE 1
    if (dyn_mode == 1)
    {
      en_gyro = false;
      if (any_ut1)
      {
        if (ut[2] || ut[3] || ut[4])
          ramp_up1 = constrain(ramp_up1 + 0.67, -base_speed, base_speed);
        left_speed = base_speed + ramp_up1 + output;
        right_speed = base_speed + ramp_up1 - output;
      }
      else
      {
        left_speed = spin_speed * last_dir;
        right_speed = -spin_speed * last_dir;
      }
    }

    // MODE 2
    else if (dyn_mode == 2)
    {
      en_gyro = false;
      if (any_ut1)
      {
        left_speed = 30 + output;
        right_speed = 30 - output;
        if (results[3].Distance > 60)
        {
          closeTime = now;
        }
        else if (now - closeTime > 500)
        {
          dyn_mode = 1;
          ramp_up1 = 30 - base_speed;
        }
      }
      else
      {
        left_speed = spin_speed * last_dir;
        right_speed = -spin_speed * last_dir;
      }
    }
    else if (dyn_mode == 3)
    {
      en_gyro = true;
      left_speed = -gyro_output;
      right_speed = gyro_output;
    }

    drive(left_speed, right_speed);
  }
  else
  {
    left_speed = 0;
    right_speed = 0;
    ramp_up1 = 0;
  }

  unsigned long loopTime = millis() - now;

  // Serial.printf("Yaw: %.2f\n", yaw);
  // printf("%lu\t(%d)%d\t(%d)%d\t(%d)%d\t(%d)%d\t(%d)%d\t(%d)%d\t(%d)%d\tmode: %d\terror: %.2f\toutput: %.2f\tlast_dir: %d\n\r", loopTime, results[0].Status, results[0].Distance, results[1].Status, results[1].Distance, results[2].Status, results[2].Distance, results[3].Status, results[3].Distance, results[4].Status, results[4].Distance, results[5].Status, results[5].Distance, results[6].Status, results[6].Distance, dyn_mode, error, output, last_dir);
}