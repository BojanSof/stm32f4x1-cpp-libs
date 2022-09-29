#include <cstdint>

#include <STM32F4x1/Gpio.hpp>
#include <STM32F4x1/Clock.hpp>

#include <Piso.hpp>
#include <Sipo.hpp>


int main()
{
  Stm32::deviceInit();
  using namespace Devices;

  PisoRegister<
    Gpio<Port::B, 0>
  , Gpio<Port::A, 1>
  , Gpio<Port::A, 2> > pisoReg;

  SipoRegister<
    Gpio<Port::A, 3>
  , Gpio<Port::A, 4>
  , Gpio<Port::A, 6>
  , Gpio<Port::A, 5>
  , Gpio<Port::A, 7> > sipoReg;

  while(true)
  {
    sipoReg.loadStates(pisoReg.readStates());
  }

  return 0;
}
