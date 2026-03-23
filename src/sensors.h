#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <VL53L1X_ULD.h>
#include <Adafruit_MCP23X08.h>
#include <Adafruit_NeoPixel.h>
#include <config.h>

void read_sensors(VL53L1X_Result_t *results);
bool init_sensor(VL53L1X_ULD &sensor, uint8_t address, uint8_t xshut);
void set_sensor_settings(VL53L1X_ULD &sensor, EDistanceMode mode, uint16_t roi_x, uint16_t roi_y, uint8_t roi_center, uint16_t timing_budget, uint16_t inter_measurement, uint16_t threshold);
void setup_sensors();
void callibrate();

extern Adafruit_MCP23X08 mcp;
extern VL53L1X_ULD sensor[sc];
extern Adafruit_NeoPixel pixels;

#endif