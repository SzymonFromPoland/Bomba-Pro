#ifndef TUNER_H
#define TUNER_H

#include <VL53L1X_ULD.h>
#include <config.h>

void startTuner(float *kp, float *kd, float *base_speed, VL53L1X_Result_t *results, float *error, float *output);

#endif
