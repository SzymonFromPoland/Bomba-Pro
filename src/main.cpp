#include <Arduino.h>
#include <VL53L1X_ULD.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MCP23X08.h>
#include <Wire.h>
#include "config.h"
#include "sensors.h"

Adafruit_NeoPixel pixels(7, leds, NEO_GRB + NEO_KHZ800);
Adafruit_MCP23X08 mcp;
VL53L1X_ULD sensor[sc];

void setup()
{
  Serial.begin(115200);
  pixels.begin();

  Wire.begin(sda, scl, 100000);

  pixels.fill(pixels.Color(0, 0, 0));
  pixels.show();

  delay(250);

  for (int i = 0; i < sc; i++)
  {
    pixels.setPixelColor(i, pixels.Color(0, 80, 0));
    pixels.show();
    delay(40);
  }

  delay(250);

  pixels.fill(pixels.Color(0, 0, 0));
  pixels.show();

  Serial.println("All sensors initialized successfully!");

  pinMode(m1a, OUTPUT);
  pinMode(m1b, OUTPUT);
  pinMode(m2a, OUTPUT);
  pinMode(m2b, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(stby, OUTPUT);

  digitalWrite(stby, HIGH);
}

void loop()
{
  VL53L1X_Result_t results;
  for (int i = 0; i < sc; i++)
  {
    sensor[i].GetResult(&results);
    uint16_t distance = (results.Status == 0) ? results.Distance : threshold;

    if (distance < threshold)
    {
      pixels.setPixelColor(i, pixels.Color(0, 0, 80));
    }
    else
    {
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    }

    pixels.show();

    Serial.printf("%d ", distance);

    sensor[i].ClearInterrupt();
  }

  Serial.println();

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