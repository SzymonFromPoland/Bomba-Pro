#ifndef CONFIG_H
#define CONFIG_H


// Variables

const uint16_t threshold = 500;
const int brightness = 200;


// MCP23008 xshut pin mapping

const int sc = 7;

const int xs1 = 7;
const int xs2 = 6;
const int xs3 = 5;
const int xs4 = 4;
const int xs5 = 3;
const int xs6 = 2;
const int xs7 = 1;

const int xshut_pins[sc] = {xs1, xs2, xs3, xs4, xs5, xs6, xs7};
const int addresses[sc] = {0x54, 0x56, 0x58, 0x5A, 0x5C, 0x5E, 0x60};

// Motor driver pin mapping
const int stby = 15;

const int pwm1 = 12;
const int m1a = 14;
const int m1b = 13;

const int pwm2 = 18;
const int m2a = 16;
const int m2b = 17;

// I2C pins
const int scl = 10;
const int sda = 11;

// Interrupt pin for MPU6050 sensor
const int interrupt = 9;

// RGB LEDs pin
const int leds = 8;

// IR receiver pin
const int rcv = 4;

// User button pin
const int btn = 1;

#endif