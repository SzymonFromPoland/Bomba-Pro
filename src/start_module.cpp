#include "start_module.h"
#include "config.h"

static RC5 rc5_global(rcv);
static TaskHandle_t IRTaskHandle;

bool hold_led = false;
bool started = false;

void irTask(void *parameter)
{
  uint8_t START, STOP;
  uint8_t ledPin = (uint8_t)(uintptr_t)parameter;

  prefs_global.begin("robot", true);
  STOP = prefs_global.getUInt("stop_address", 0);
  START = prefs_global.getUInt("start_address", 0);
  prefs_global.end();

  for (;;)
  {
    unsigned char toggle;
    unsigned char address;
    unsigned char command;

    if (rc5_global.read(&toggle, &address, &command))
    {
      if (address == 0x0B)
      {
        START = command + 1;
        STOP = command;

        prefs_global.begin("robot", false);
        prefs_global.putUInt("stop_address", STOP);
        prefs_global.putUInt("start_address", START);
        prefs_global.end();

        hold_led = true;
        for (int i = 0; i < 5; i++)
        {
          digitalWrite(ledPin, HIGH);
          vTaskDelay(50 / portTICK_PERIOD_MS);
          digitalWrite(ledPin, LOW);
          vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        hold_led = false;
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }
      else if (address == 0x07)
      {
        if (command == START)
          started = true;
        else if (command == STOP)
          started = false;
      }
    }
  }
}

void startIRTask(uint8_t pin, uint8_t ledPin)
{
  rc5_global = RC5(pin);
  xTaskCreatePinnedToCore(irTask, "IR_Task", 8192, (void *)(uintptr_t)ledPin, 1, &IRTaskHandle, 1);
}
