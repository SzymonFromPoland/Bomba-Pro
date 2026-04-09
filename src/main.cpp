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

int mode = 1;
int dyn_mode = 1;

float base_speed = 67;
int last_dir = -1;

float error = 0;
float output = 0;
VL53L1X_Result_t results[sc];
Adafruit_MPU6050 mpu;

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

  startTuner(&Kp, &Kd, &base_speed, results, &error, &output);
}

float prev_error = 0;
float integral = 0;
float pd(float error, float dt, float Kp, float Kd)
{
  static float prev_error = 0.0f;
  static float prev_derivative = 0.0f;
  if (dt < 0.001f)
    dt = 0.001f;
  float P = Kp * error;
  float derivative = (error - prev_error) / dt;
  derivative = prev_derivative * derivative;

  prev_derivative = derivative;
  prev_error = error;

  float D = Kd * derivative;

  float output = P + D;

  return output;
}

float dt;
unsigned long lastTime = 0;
unsigned long spinStart = 0;
unsigned long lastSawTime = 0;

float yaw = 0;

void loop()
{
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  handle_mode();

  digitalWrite(stby, started);

  bool ut[sc];
  read_sensors(results, &error, ut);

  if (error < -0.01)
    last_dir = -1;
  else if (error > 0.1)
    last_dir = 1;

  output = pd(error, dt, Kp, Kd);

  bool aat = std::none_of(ut, ut + sc, [](bool b)
                          { return b; });

  if (aat)
  {
    if (spinStart == 0)
      spinStart = millis();
    bool slowSpin = (millis() - lastSawTime < 1000);
    float spin_speed = (millis() - spinStart < 34) ? 100 : (slowSpin ? 45 : 50);
    drive(spin_speed * last_dir, -spin_speed * last_dir);
  }
  else if (dyn_mode == 1)
  {
    spinStart = 0;
    lastSawTime = millis();
    if (ut[2] || ut[3] || ut[4])
      base_speed = constrain(base_speed + 0.67, 0, 100);
    drive(base_speed + output, base_speed - output);
  }
  else if (dyn_mode == 2)
  {
    spinStart = 0;
    lastSawTime = millis();
    base_speed = constrain(base_speed + 0.25, 0, 30);
    drive(base_speed + output, base_speed - output);
    if (results[3].Distance < 40 && started)
      dyn_mode = 1;
    base_speed = 30;
  }

  unsigned long loopTime = millis() - now;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float rate = (g.gyro.z) * RAD_TO_DEG;
  yaw += rate * dt;

  Serial.printf("Yaw: %.2f\n", yaw);

  // printf("%lu\t(%d)%d\t(%d)%d\t(%d)%d\t(%d)%d\t(%d)%d\t(%d)%d\t(%d)%d\tmode: %d\terror: %.2f\toutput: %.2f\tlast_dir: %d\n\r", loopTime, results[0].Status, results[0].Distance, results[1].Status, results[1].Distance, results[2].Status, results[2].Distance, results[3].Status, results[3].Distance, results[4].Status, results[4].Distance, results[5].Status, results[5].Distance, results[6].Status, results[6].Distance, dyn_mode, error, output, last_dir);
}