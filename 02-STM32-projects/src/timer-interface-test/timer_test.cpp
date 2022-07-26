// Software PWM using timer interrupts
#include <Gpio.hpp>
#include <Clock.hpp>
#include <GeneralPurposeTimer.hpp>

using namespace Stm32;


// std::function<void()> setPinHigh()
// {
//   Gpio<Port::A, 3> pin;
//   pin.setLevel(true);
// };

// std::function<void()> setPinLow()
// {
//   Gpio<Port::A, 3> pin;
//   pin.setLevel(false);
//   };


int main()
{
  deviceInit();
  Gpio<Port::A, 6> pin;
  pin.setAlternateFunction(GpioAlternateFunction::Af2);
  auto& timer = GeneralPurposeTimer<3>::getInstance(); // zosto imame referenca? a mozda i zna
  using PwmConfig = PwmModeConfig<1000,  // period = 1000 us
                                  200,   // high level = 200 us
                                  1,     // capture/compare channel 1
                                  true,
                                  false>;
  timer.setMode<PwmConfig>();
  //soft pwm
  //timer.enableOverflowInterrupt([&pin](){ pin.setLevel(true); });
  //timer.enableCaptureCompareInterrupt<1>([&pin](){ pin.setLevel(false); });

  timer.start();
  while(true);
  
 
  return 0;
}
