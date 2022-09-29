#include <Gpio.hpp>
#include <Clock.hpp>
#include <CycleCounter.hpp>

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
    CycleCounter::delay<>(1s);
    ledPin.setLevel(false);
    CycleCounter::delay<>(1s);
  }

  return 0;
}
