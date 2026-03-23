#include <Arduino.h>
#include <VL53L1X_ULD.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MCP23X08.h>
#include <Wire.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"

#include <flag_detector_inferencing.h>

Adafruit_NeoPixel pixels(7, leds, NEO_GRB + NEO_KHZ800);
Adafruit_MCP23X08 mcp;
VL53L1X_ULD sensor[sc];

float features[2];

int get_data(size_t offset, size_t length, float *out_ptr)
{
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}

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

const uint32_t LOOP_PERIOD_US = 10000; // 10 ms = 100 Hz
uint32_t lastLoopTime = 0;

void loop()
{
  uint32_t now = micros();

  if (now - lastLoopTime >= LOOP_PERIOD_US)
  {
    lastLoopTime += LOOP_PERIOD_US; // stable drift-free timing

    VL53L1X_Result_t results[sc];
    read_sensors(results);

    Serial.printf("%d, %d, %d, %d\n\r",
                  results[3].Distance,
                  results[3].SigPerSPAD,
                  results[3].NumSPADs,
                  results[3].Status);

    // ---- ML part (unchanged, ready for 100Hz inference) ----
    /*
    features[0] = results[3].Distance;
    features[1] = results[3].SigPerSPAD;

    signal_t signal;
    signal.total_length = 2;
    signal.get_data = get_data;

    ei_impulse_result_t result;

    if (run_classifier(&signal, &result, false) == EI_IMPULSE_OK)
    {
      String classified =
        (result.classification[0].value > result.classification[1].value)
        ? "NO_FLAG"
        : "YES_FLAG";

      Serial.printf("Classified: %s\n", classified.c_str());
    }
    */
  }
}