#include "sensors.h"

int16_t offsets[7] = {1, 1, -58, 2, -32, -24, -18};
uint16_t xtalks[7] = {61831, 40635, 2808, 60635, 1970, 42614, 53359};

float features[3];

int get_data(size_t offset, size_t length, float *out_ptr)
{
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

bool init_sensor(VL53L1X_ULD &sensor, uint8_t address, uint8_t xshut)
{
    delay(50);
    mcp.digitalWrite(xshut, HIGH);
    delay(50);
    VL53L1_Error result = sensor.Begin(0x29);
    if (result != VL53L1_ERROR_NONE)
        return false;

    result = sensor.SetI2CAddress(address);
    return true;
}

void set_sensor_settings(VL53L1X_ULD &sensor, EDistanceMode mode, uint16_t roi_x, uint16_t roi_y, uint8_t roi_center, uint16_t timing_budget, uint16_t inter_measurement, uint16_t threshold, int16_t xtalk, int16_t offset)
{
    sensor.SetDistanceMode(mode);
    sensor.SetROI(roi_x, roi_y);
    sensor.SetROICenter(roi_center);
    sensor.SetTimingBudgetInMs(timing_budget);
    sensor.SetInterMeasurementInMs(inter_measurement);
    sensor.SetInterruptPolarity(ActiveLOW);
    sensor.SetDistanceThreshold(0, threshold, Out);
    // sensor.SetXTalk(xtalk);          // Where is your xtalk? Dont need it...
    // sensor.SetOffsetInMm(offset);
    sensor.StartRanging();
}

void setup_sensors()
{
    for (int i = 0; i < 10; i++)
    {
        Wire.beginTransmission(0x20);
        Wire.endTransmission();
        delay(10);
    }

    if (!mcp.begin_I2C(0x20, &Wire))
    {
        Serial.println("Error: MCP23008 not found!");
        pixels.fill(pixels.Color(80, 0, 0));
        pixels.show();
        while (1)
            ;
    }

    for (int i = 0; i < 8; i++)
    {
        mcp.pinMode(i, OUTPUT);
        mcp.digitalWrite(i, LOW);
    }

    for (int i = 0; i < sc; i++)
    {
        if (!init_sensor(sensor[i], addresses[i], xshut_pins[i]))
        {
            Serial.printf("Error initializing sensor %d at address 0x%02X\n", i, addresses[i]);
            pixels.setPixelColor(i, pixels.Color(brightness, 0, 0));
            pixels.show();
            while (1)
                ;
        }

        set_sensor_settings(sensor[i], Short, 11, 4, 61, 15, 15, threshold, xtalks[i], offsets[i]); // roi 9x4 center 61
        pixels.setPixelColor(i, pixels.Color(0, brightness, 0));
        pixels.show();
        Serial.printf("Sensor %d initialized and moved to 0x%02X\n", i, sensor[i].GetI2CAddress());
    }

    pixels.clear();
    pixels.show();
}

void read_sensors(VL53L1X_Result_t *results, bool ignoreFlags)
{
    for (int i = 0; i < sc; i++)
    {
        sensor[i].GetResult(&results[i]);
        uint16_t distance = (results[i].Status == 0) ? min(results[i].Distance, threshold) : threshold;

        if (!hold_led)
            pixels.setPixelColor(i, pixels.Color(0, 0, abs(map(threshold - distance, 0, threshold, 0, brightness))));

        if (!hold_led)
            pixels.show();

        sensor[i].ClearInterrupt();
    }
}

void callibrate()
{
    pixels.clear();
    pixels.show();

    for (int i = 0; i < sc; i++)
    {
        pixels.setPixelColor(i, pixels.Color(0, 50, 0));
        pixels.show();

        Serial.println("Press button");
        while (digitalRead(btn))
            ;
        Serial.println("Callibration started.");
        delay(600);
        pixels.setPixelColor(i, pixels.Color(50, 50, 0));
        pixels.show();
        int16_t foundOffset;
        VL53L1_Error status2 = sensor[i].CalibrateOffset(140, &foundOffset);
        Serial.println("Calibrated offset: " + String(foundOffset));
        uint16_t foundXTalk;
        VL53L1_Error status1 = sensor[i].CalibrateXTalk(140, &foundXTalk);
        Serial.println("Calibrated XTalk: " + String(foundXTalk));

        while (!digitalRead(btn))
            ;
    }
}