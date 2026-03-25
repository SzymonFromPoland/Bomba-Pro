#include <Arduino.h>
#include <VL53L1X_ULD.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MCP23X08.h>
#include <Wire.h>
#include <config.h>
#include <sensors.h>
#include <motors.h>
#include <start_module.h>

Adafruit_NeoPixel pixels(7, leds, NEO_GRB + NEO_KHZ800);
Adafruit_MCP23X08 mcp;
VL53L1X_ULD sensor[sc];

Preferences prefs_global;

float Kp = 100.0;
float Kd = 35.0;

int mode = 1;

int base_speed = 100;
int last_dir = -1;

volatile float error = 0;
VL53L1X_Result_t results[sc];
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void setup()
{
  Serial.begin(115200);
  pixels.begin();
  Wire.begin(sda, scl, 800000);

  setup_motors();
  setup_sensors();
  startIRTask((uint8_t)rcv);

  pinMode(btn, INPUT_PULLUP);
}

void increment_mode()
{
  pixels.clear();
  mode = (mode + 1 > 3) ? 1 : mode + 1;
  for (int i = 0; i < mode; i++)
    pixels.setPixelColor(i, pixels.Color(75, 50, 0));
  pixels.show();
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
unsigned long loopStart = 0;

void loop()
{
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  loopStart = millis();

  if (!digitalRead(btn) && !started)
  {
    hold_led = true;
    increment_mode();
    while (!digitalRead(btn))
      delay(10);
    delay(600);
    hold_led = false;
  }

  digitalWrite(stby, started);

  VL53L1X_Result_t results[sc];
  float error;
  float output;
  read_sensors(results, &error);
  output = pd(error, dt, Kp, Kd);

  bool aat =
      (results[0].Distance == threshold) &&
      (results[1].Distance == threshold) &&
      (results[2].Distance == threshold) &&
      (results[3].Distance == threshold) &&
      (results[4].Distance == threshold) &&
      (results[5].Distance == threshold) &&
      (results[6].Distance == threshold);

  if (aat)
    drive(base_speed, -base_speed);
  else
    drive(base_speed + round(output), base_speed - round(output));

  unsigned long loopTime = millis() - loopStart;

  printf("%lu\t%d\t%d\t%d\t%d\t%d\t%d\t%d\tmode: %d\terror: %.2f\toutput: %.2f\n\r", loopTime, results[0].Distance, results[1].Distance, results[2].Distance, results[3].Distance, results[4].Distance, results[5].Distance, results[6].Distance, mode, error, output);
}