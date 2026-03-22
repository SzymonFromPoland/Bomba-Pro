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
