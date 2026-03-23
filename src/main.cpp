#include <Arduino.h>
#include <VL53L1X_ULD.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MCP23X08.h>
#include <Wire.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"

Adafruit_NeoPixel pixels(7, leds, NEO_GRB + NEO_KHZ800);
Adafruit_MCP23X08 mcp;
VL53L1X_ULD sensor[sc];

void setup()
{
  Serial.begin(115200);
  // delay(1500);
  pixels.begin();
  Wire.begin(sda, scl, 400000);

  setup_motors();
  setup_sensors();

  pinMode(btn, INPUT_PULLUP);
}

void loop()
{
  VL53L1X_Result_t result[sc];
  read_sensors(result);

  // if (!digitalRead(btn))
  //   callibrate();

  // drive(100, 100);
  // delay(1000);

  // // backward
  // drive(-100, -100);
  // delay(1000);

  // // spin right
  // drive(100, -100);
  // delay(1000);

  // // spin left
  // drive(-100, 100);
  // delay(1000);

  // // gentle forward
  // drive(50, 50);
  // delay(1000);

  // // curve right
  // drive(100, 50);
  // delay(1000);

  // // curve left
  // drive(50, 100);
  // delay(1000);

  // // stop
  // drive(0, 0);
  // delay(1000);
}