#include <cstdio>
#include <chrono>
#include <type_traits>

#include <STM32F4x1/Gpio.hpp>
#include <STM32F4x1/Clock.hpp>

#include <FreeRTOS.h>
#include <task.h>

void task1(void* param)
{
  (void)param;
  using namespace Stm32;
  Pins::PC14 ledPin;
  ledPin.setMode(GpioMode::Output);
  while(true)
  {
    ledPin.setLevel(true);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ledPin.setLevel(false);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void task2(void* param)
{
  (void)param;
  using namespace Stm32;
  Pins::PC15 ledPin;
  ledPin.setMode(GpioMode::Output);
  while(true)
  {
    ledPin.setLevel(false);
    vTaskDelay(pdMS_TO_TICKS(500));
    ledPin.setLevel(true);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

int main()
{
  using namespace Stm32;
  deviceInit();
  
  xTaskCreate(task1, "task1", 100, NULL, tskIDLE_PRIORITY + 1, NULL);
  xTaskCreate(task2, "task2", 100, NULL, tskIDLE_PRIORITY + 1, NULL);
  vTaskStartScheduler();
  
  while(true);

  return 0;
}
extern "C"
{
  void vApplicationTickHook(void)
  {
  }

  void vApplicationIdleHook(void)
  {
  }
  
  void vApplicationMallocFailedHook(void)
  {
    taskDISABLE_INTERRUPTS();
    while(true);
  }

  void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
  {
    (void) pcTaskName;
    (void) pxTask;

    taskDISABLE_INTERRUPTS();
    while(true);
  }
}