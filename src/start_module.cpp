#include "start_module.h"
#include "config.h"

static RC5 rc5_global(rcv);
static TaskHandle_t IRTaskHandle;

bool hold_led = false;
bool started = false;

void irTask(void *parameter)
{
  uint8_t START, STOP;

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
        for (int i = 0; i < 7; i++)
        {
          pixels.fill(pixels.Color(255, 255, 255));
          pixels.show();
          vTaskDelay(30 / portTICK_PERIOD_MS);
          pixels.clear();
          pixels.show();
          vTaskDelay(30 / portTICK_PERIOD_MS);
        }
        hold_led = false;
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }
      else if (address == 0x07)
      {
        if (command == START)
          started = true;
        else if (command == STOP)
        {
          started = false;
          dyn_mode = mode;
        }
      }
    }
  }
}

void startIRTask(uint8_t pin)
{
  rc5_global = RC5(pin);
  xTaskCreatePinnedToCore(irTask, "IR_Task", 8192, nullptr, 1, &IRTaskHandle, 1);
}
