// Software PWM using timer interrupts
#include <STM32F4x1/Gpio.hpp>
#include <STM32F4x1/Clock.hpp>
#include <STM32F4x1/GeneralPurposeTimer.hpp>
#include <STM32F4x1/CycleCounter.hpp>

#include <SevenSegmentDisplay.hpp>

using namespace Stm32;

int main()
{
  deviceInit();

  using TriggerPin = Pins::PA6;
  using EchoPin = Pins::PB0;

  auto& timer = GeneralPurposeTimer<3>::getInstance();

  using PwmConfig = PwmModeConfig<50000,
                                  10,
                                  true,
                                  false,
                                  TriggerPin>;
  timer.configureCaptureCompareChannel<1, PwmConfig>();
 

  using CaptureConfig1 = InputCaptureConfig<50000,
                                            TimerCaptureCompareSelection::InputTiChannel,
                                            TimerDigitalFilter::NoFilter,
                                            TimerCapturePolarity::RisingEdge,
                                            TimerCapturePrescaler::Div1,
                                            EchoPin>;

  using CaptureConfig2 = InputCaptureConfig<50000,
                                            TimerCaptureCompareSelection::InputTiOther,
                                            TimerDigitalFilter::NoFilter,
                                            TimerCapturePolarity::FallingEdge,
                                            TimerCapturePrescaler::Div1,
                                            EchoPin>;

  timer.configureCaptureCompareChannel<3, CaptureConfig1>();
  timer.configureCaptureCompareChannel<4, CaptureConfig2>();

  volatile uint32_t risingCapture = 0, travelTime = 0;
  volatile bool distanceMeasured = false;
  
  Devices::SevenSegmentDisplay< 
                       Gpio<Port::A, 8>
                    ,  Gpio<Port::A, 9>
                    ,  Gpio<Port::A, 10>
                    ,  Gpio<Port::A, 11>
                    ,  Gpio<Port::A, 12>
                    ,  Gpio<Port::A, 15>
                    ,  Gpio<Port::B, 3>
                    ,  Gpio<Port::B, 6>
                    ,  Gpio<Port::B, 5> > sseg;

  timer.enableCaptureCompareInterrupt<3>([&timer, &risingCapture](){
    risingCapture = timer.getCaptureCompareTime<3>();
  });
  timer.enableCaptureCompareInterrupt<4>([&timer, &travelTime, &risingCapture, &distanceMeasured](){
    travelTime = timer.getCaptureCompareTime<4>() - risingCapture;
    distanceMeasured = true;
  });
  
  float speed = 0.034;
  timer.start();
  uint32_t ssegPrevRefreshTime = 0, currentTime = 0;
  while(true)
  {
    if(distanceMeasured)
    {
      sseg.setNumber(static_cast<uint8_t>(travelTime * speed / 2));
    }
    currentTime = timer.getCountTime();
    if(currentTime - ssegPrevRefreshTime > 10000)
    {
      sseg.update();
      ssegPrevRefreshTime = currentTime;
    }
 }
  
 
  return 0;
}
