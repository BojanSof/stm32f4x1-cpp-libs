#include <stm32f4xx.h>
#include <STM32F4x1/Clock.hpp>
#include <STM32F4x1/Gpio.hpp>
#include <tusb.h>

int main()
{
  using namespace Stm32;
  deviceInit();
  
  // configure USB pins
  /* Configure USB D+ D- Pins */
  Pins::setMode<Pins::PA11, Pins::PA12>(GpioMode::Output);
  Pins::setOutputType<Pins::PA11, Pins::PA12>(GpioOutputType::PushPull);
  Pins::setPullType<Pins::PA11, Pins::PA12>(GpioPullType::None);
  Pins::setSpeed<Pins::PA11, Pins::PA12>(GpioSpeed::High);
  Pins::PA11{}.setAlternateFunction(Pins::PA11::AlternateFunctions::OTG_FS_DM);
  Pins::PA12{}.setAlternateFunction(Pins::PA12::AlternateFunctions::OTG_FS_DP);

  /* Configure VBUS Pin */
  Pins::PA9 vbusPin;
  vbusPin.setMode(GpioMode::Input);
  vbusPin.setPullType(GpioPullType::None);
  vbusPin.setAlternateFunction(Pins::PA9::AlternateFunctions::OTG_FS_VBUS);

  /* ID Pin */
  Pins::PA10 idPin;
  idPin.setMode(GpioMode::Output);
  idPin.setOutputType(GpioOutputType::OpenDrain);
  idPin.setPullType(GpioPullType::PullUp);
  idPin.setSpeed(GpioSpeed::High);
  idPin.setAlternateFunction(Pins::PA10::AlternateFunctions::OTG_FS_ID);

  // enalbe USB clock
  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
  // configure USB
  USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
  USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
  USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;

  tusb_init();
  while(true)
  {
    tud_task();
  }
}

extern "C"
{
  void OTG_FS_IRQHandler(void)
  {
    tud_int_handler(BOARD_DEVICE_RHPORT_NUM);
  }
}