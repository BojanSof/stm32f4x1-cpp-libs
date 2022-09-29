#include <cstdint>

#include <STM32F4x1/Gpio.hpp>
#include <STM32F4x1/Clock.hpp>

int main()
{
  using namespace Stm32;
  deviceInit();
  Gpio<Port::C, 13> ledPin;
  ledPin.setMode(GpioMode::Output);
  while(true)
  {
    ledPin.setLevel(true);
    for(uint32_t i = 0; i < 1000000; ++i);  //< some delay
    ledPin.setLevel(false);
    for(uint32_t i = 0; i < 1000000; ++i);  //< some delay
  }

  return 0;
}
