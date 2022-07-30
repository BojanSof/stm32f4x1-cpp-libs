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
  Gpio<Port::A, 6> triggerPin;
  triggerPin.setAlternateFunction(GpioAlternateFunction::Af2);
  Gpio<Port::B, 0> echoPin;
  echoPin.setAlternateFunction(GpioAlternateFunction::Af2);
  auto& timer = GeneralPurposeTimer<3>::getInstance();
  using PwmConfig = PwmModeConfig<50000,
                                  10,
                                  1,
                                  true,
                                  false>;
  timer.configureCaptureCompareChannel<PwmConfig>();
  //soft pwm
  //timer.enableOverflowInterrupt([&pin](){ pin.setLevel(true); });
  //timer.enableCaptureCompareInterrupt<1>([&pin](){ pin.setLevel(false); });

  using CaptureConfig1 = InputCaptureConfig<50000,
                                            3,
                                            TimerCaptureCompareSelection::InputTiChannel,
                                            TimerDigitalFilter::NoFilter,
                                            TimerCapturePolarity::RisingEdge,
                                            TimerCapturePrescaler::Div1>;

  using CaptureConfig2 = InputCaptureConfig<50000,
                                            4,
                                            TimerCaptureCompareSelection::InputTiOther,
                                            TimerDigitalFilter::NoFilter,
                                            TimerCapturePolarity::FallingEdge,
                                            TimerCapturePrescaler::Div1>;

  timer.configureCaptureCompareChannel<CaptureConfig1>();
  timer.configureCaptureCompareChannel<CaptureConfig2>();

  volatile uint16_t risingCapture = 0, fallingCapture = 0;
  volatile bool distanceMeasured = false;
  timer.enableCaptureCompareInterrupt<3>([&risingCapture](){
    risingCapture = TIM3->CCR3;
  });
  timer.enableCaptureCompareInterrupt<4>([&fallingCapture, &distanceMeasured](){
    fallingCapture = TIM3->CCR4;
    distanceMeasured = true;
  });
  
  uint16_t timeDiff = 0;
  float speed = 0.034;
  timer.start();
  while(true)
  {
    if(distanceMeasured)
    {
      timeDiff = fallingCapture - risingCapture;
      distanceMeasured = false;
    }
  }
  
 
  return 0;
}
