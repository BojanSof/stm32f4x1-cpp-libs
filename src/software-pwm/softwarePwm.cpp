#include <STM32F4x1/Gpio.hpp>
#include <STM32F4x1/Clock.hpp>
#include <STM32F4x1/CycleCounter.hpp>

int main()
{
  using namespace Stm32;
  deviceInit();
  Gpio<Port::C, 13> ledPin;
  ledPin.setMode(GpioMode::Output);
  using namespace std::chrono_literals;
  
  while(true)
  {
    ledPin.setLevel(true);
    CycleCounter::delay(1s);
    ledPin.setLevel(false);
    CycleCounter::delay(1s);
  }

  return 0;
}
