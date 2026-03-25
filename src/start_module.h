#ifndef START_MODULE_H
#define START_MODULE_H

#include <Arduino.h>
#include <RC5.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>

extern bool started;
extern bool hold_led;
extern Preferences prefs_global;
extern Adafruit_NeoPixel pixels;

void startIRTask(uint8_t pin);

#endif
