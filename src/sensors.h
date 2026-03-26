#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <VL53L1X_ULD.h>
#include <Adafruit_MCP23X08.h>
#include <Adafruit_NeoPixel.h>
#include <config.h>

/*!

    @brief this function reads information from sensors.
    @param results sensor data as a list (ex. results[0])
*/
void read_sensors(VL53L1X_Result_t *results, float *error, bool *dist_ut);
bool init_sensor(VL53L1X_ULD &sensor, uint8_t address, uint8_t xshut, int index);
void set_sensor_settings(VL53L1X_ULD &sensor, VL53L1X_Settings settings);
void change_settings(VL53L1X_Settings settings);

void setup_sensors();
void callibrate();

extern Adafruit_MCP23X08 mcp;
extern VL53L1X_ULD sensor[sc];
extern Adafruit_NeoPixel pixels;

extern bool hold_led;

#endif