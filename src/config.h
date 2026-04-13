#ifndef CONFIG_H
#define CONFIG_H

#include <VL53L1X_ULD.h>

typedef struct
{
    EDistanceMode mode;
    uint16_t roi_x;
    uint16_t roi_y;
    uint8_t roi_center;
    uint16_t timing_budget;
    uint16_t inter_measurement;
    uint16_t threshold;

} VL53L1X_Settings;

const uint16_t threshold = 567;
const int brightness = 200;

const int sc = 7;

const int xs1 = 7;
const int xs2 = 6;
const int xs3 = 5;
const int xs4 = 4;
const int xs5 = 3;
const int xs6 = 2;
const int xs7 = 1;

const bool sensor_enabled[sc] = {1, 1, 1, 1, 1, 1, 1};

const int xshut_pins[sc] = {xs1, xs2, xs3, xs4, xs5, xs6, xs7};
const int addresses[sc] = {0x54, 0x56, 0x58, 0x5A, 0x5C, 0x5E, 0x60};

const VL53L1X_Settings def = {Short, 14, 4, 61, 15, 15, threshold};
const VL53L1X_Settings medium = {Long, 14, 4, 61, 20, 20, threshold};

const int stby = 15;

const int pwm1 = 12;
const int m1a = 14;
const int m1b = 13;

const int pwm2 = 18;
const int m2a = 16;
const int m2b = 17;

const int scl = 10;
const int sda = 11;

const int interrupt = 9;
const int leds = 8;
const int rcv = 4;
const int btn = 1;

#endif