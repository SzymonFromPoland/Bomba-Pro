#include "sensors.h"

int xtalks[7] = {-1, -1, -1, -1, -1, -1, -1};

bool init_sensor(VL53L1X_ULD &sensor, uint8_t address, uint8_t xshut, int index)
{
    delay(10);
    mcp.digitalWrite(xshut, HIGH);
    delay(10);
    VL53L1_Error result = sensor.Begin(0x29);
    if (result != VL53L1_ERROR_NONE)
        return false;

    result = sensor.SetI2CAddress(address);
    if (xtalks[index] != -1)
        sensor.SetXTalk((uint16_t)xtalks[index]);
    return true;
}

void set_sensor_settings(VL53L1X_ULD &sensor, VL53L1X_Settings settings)
{
    sensor.SetDistanceMode(settings.mode);
    sensor.SetROI(settings.roi_x, settings.roi_y);
    sensor.SetROICenter(settings.roi_center);
    sensor.SetTimingBudgetInMs(settings.timing_budget);
    sensor.SetInterMeasurementInMs(settings.inter_measurement);
    sensor.SetInterruptPolarity(ActiveLOW);
    sensor.SetDistanceThreshold(0, threshold, Out);
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
        pixels.fill(pixels.Color(50, 0, 0));
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
        if (!sensor_enabled[i])
            continue;

        if (!init_sensor(sensor[i], addresses[i], xshut_pins[i], i))
        {
            Serial.printf("Error initializing sensor %d at address 0x%02X\n", i, addresses[i]);
            pixels.setPixelColor(i, pixels.Color(30, 0, 0));
            pixels.show();
            while (1)
                ;
        }

        set_sensor_settings(sensor[i], def);
        pixels.setPixelColor(i, pixels.Color(0, 30, 0));
        pixels.show();
        Serial.printf("Sensor %d initialized and moved to 0x%02X\n", i, sensor[i].GetI2CAddress());
    }
    pixels.clear();
    pixels.show();
}

void change_settings(VL53L1X_Settings settings)
{
    for (int i = 0; i < sc; i++)
    {
        sensor[i].StopRanging();
        set_sensor_settings(sensor[i], settings);
    }
}

void read_sensors(VL53L1X_Result_t *results, float *error, bool *dist_ut)
{
    float eps = 1e-3f;

    float numerator = 0.0f;
    float denominator = 0.0f;

    for (int i = 0; i < sc; i++)
    {
        if (!sensor_enabled[i])
        {
            results[i].Distance = (uint16_t)threshold;
            results[i].Status = 255;
            dist_ut[i] = false;
            continue;
        }
        sensor[i].GetResult(&results[i]);
        results[i].Distance = (results[i].Status == 0) ? min(results[i].Distance, (uint16_t)threshold) : (uint16_t)threshold;
        dist_ut[i] = results[i].Distance < (uint16_t)threshold;

        float s = 1.0f / (results[i].Distance + eps);

        int position = i - 3;
        numerator += s * position;
        denominator += s;

        if (!hold_led)
        {
            pixels.setPixelColor(i, pixels.Color(0, 0, abs(map((uint16_t)threshold - results[i].Distance, 0, (uint16_t)threshold, 0, brightness))));
        }
    }

    pixels.show();

    *error = (denominator > 0.0001f) ? (numerator / denominator) : 0.0f;
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
        VL53L1_Error status2 = sensor[i].CalibrateOffset(150, &foundOffset);
        Serial.println("Calibrated offset: " + String(foundOffset));
        uint16_t foundXTalk;
        VL53L1_Error status1 = sensor[i].CalibrateXTalk(150, &foundXTalk);
        Serial.println("Calibrated XTalk: " + String(foundXTalk));

        while (!digitalRead(btn))
            ;
    }
}