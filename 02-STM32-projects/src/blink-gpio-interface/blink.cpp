// simple CMSIS only stm32f401 blink example
#include <Gpio.hpp>
#include <Clock.hpp>
#include <cstdint>

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
