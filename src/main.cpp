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

void setup()
{
  Serial.begin(115200);
  pixels.begin();
  Wire.begin(sda, scl, 400000);

  setup_motors();
  setup_sensors();
  startIRTask((uint8_t)rcv);

  pinMode(btn, INPUT_PULLUP);
}

void loop()
{
  digitalWrite(stby, started);
  VL53L1X_Result_t results[sc];
  read_sensors(results);
  printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\n\r", results[0].Distance, results[1].Distance, results[2].Distance, results[3].Distance, results[4].Distance, results[5].Distance, results[6].Distance);

}