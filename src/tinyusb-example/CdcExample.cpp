#include <STM32F4x1/Clock.hpp>
#include <tusb.h>

int main()
{
  // ensure Arduino UNO runs the SPI slave sketch included in this file's directory
  using namespace Stm32;
  deviceInit();
  // enalbe USB clock
  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFS_EN;
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
    return;
  }
}