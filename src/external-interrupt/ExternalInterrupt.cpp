#include <Gpio.hpp>
#include <Clock.hpp>
#include <CycleCounter.hpp>

#include <iostream>

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