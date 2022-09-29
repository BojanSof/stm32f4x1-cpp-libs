#include <STM32F4x1/Gpio.hpp>
#include <STM32F4x1/Clock.hpp>
#include <STM32F4x1/CycleCounter.hpp>

int main()
{
  using namespace Stm32;
  deviceInit();
  Gpio<Port::C, 13> ledPin;
  ledPin.setMode(GpioMode::Output);
  Pins::PC14 buttonPin;
  buttonPin.setMode(GpioMode::Input);
  bool lvl = true;
  buttonPin.enableInterrupt<GpioExternalInterruptEdge::Rising>([&]() {
    ledPin.setLevel(lvl);
    lvl = !lvl;
  } );

  while(true);

  return 0;
}