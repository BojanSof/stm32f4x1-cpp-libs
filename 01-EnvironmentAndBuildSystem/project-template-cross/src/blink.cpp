// simple CMSIS only stm32f401 blink example
#include <stm32f4xx.h>

#include <cstdint>

int main()
{
  constexpr uint8_t ledPin = 13;  //< LED on PC13
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //< clock for GPIOC
  GPIOE->MODER = 0x1 << (2*ledPin);  //< general purpose output mode

  while(true)
  {
    GPIOE->ODR |= 1 << ledPin;
    for(uint32_t i = 0; i < 10000000; ++i);  //< some delay
    GPIOE->ODR &= ~(1 << ledPin);
    for(uint32_t i = 0; i < 10000000; ++i);  //< some delay
  }

  return 0;
}
