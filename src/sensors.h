#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <VL53L1X_ULD.h>
#include <Adafruit_MCP23X08.h>

bool init_sensor(VL53L1X_ULD &sensor, uint8_t address, uint8_t xshut);
void set_sensor_settings(VL53L1X_ULD &sensor, EDistanceMode mode, uint16_t roi_x, uint16_t roi_y, uint8_t roi_center, uint16_t timing_budget, uint16_t inter_measurement, uint16_t threshold);

extern Adafruit_MCP23X08 mcp;

#endif