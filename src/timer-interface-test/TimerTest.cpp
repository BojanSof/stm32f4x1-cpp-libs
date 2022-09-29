// Software PWM using timer interrupts
#include <Gpio.hpp>
#include <Clock.hpp>
#include <GeneralPurposeTimer.hpp>

using namespace Stm32;


int main()
{
  deviceInit();
  auto& timer = GeneralPurposeTimer<3>::getInstance();
  using PwmConfig = PwmModeConfig<50000,
                                  10,
                                  true,
                                  false,
                                  Pins::PA6>;
  timer.configureCaptureCompareChannel<1, PwmConfig>();
  //soft pwm
  //timer.enableOverflowInterrupt([&pin](){ pin.setLevel(true); });
  //timer.enableCaptureCompareInterrupt<1>([&pin](){ pin.setLevel(false); });

  timer.start();
  while(true);
 
  return 0;
}
