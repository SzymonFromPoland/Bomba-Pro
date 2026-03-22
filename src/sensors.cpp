#include "sensors.h"

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

void set_sensor_settings(VL53L1X_ULD &sensor, EDistanceMode mode, uint16_t roi_x, uint16_t roi_y, uint8_t roi_center, uint16_t timing_budget, uint16_t inter_measurement, uint16_t threshold)
{
    sensor.SetDistanceMode(mode);
    sensor.SetROI(roi_x, roi_y);
    sensor.SetROICenter(roi_center);
    sensor.SetTimingBudgetInMs(timing_budget);
    sensor.SetInterMeasurementInMs(inter_measurement);
    sensor.SetInterruptPolarity(ActiveLOW);
    sensor.SetDistanceThreshold(0, threshold, Out);
    sensor.StartRanging();
}

void init_sensors()
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
            pixels.setPixelColor(i, pixels.Color(80, 0, 0));
            pixels.show();
            while (1)
                ;
        }

        set_sensor_settings(sensor[i], Short, 11, 5, 58, 15, 15, threshold);
        pixels.setPixelColor(i, pixels.Color(0, 80, 0));
        pixels.show();
        Serial.printf("Sensor %d initialized and moved to 0x%02X\n", i, sensor[i].GetI2CAddress());
    }
}