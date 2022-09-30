#include <stm32f4xx.h>
#include "STM32F4x1/Clock.hpp"


namespace Stm32
{
  void deviceInit()
  {
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;          // CPU cannot fetch the instruction with 84 MHZ from the flash
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;             // APB1 prescaler = 2 (Max frequency allowed 42 MHz)
    RCC->CR |= RCC_CR_HSEON;                      // Enable external oscillator
    while(!(RCC->CR & RCC_CR_HSERDY));            // Wait until clock gets stabilized

    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC;           // HSE is the source for main PLL
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
    RCC->PLLCFGR |= 25 << RCC_PLLCFGR_PLLM_Pos;   // PLL divisor = 25
    RCC->PLLCFGR |= 336 << RCC_PLLCFGR_PLLN_Pos;  // PLL multiplier = 336
    RCC->PLLCFGR |= 0x1 << RCC_PLLCFGR_PLLP_Pos;  // PLL output divisor = 4
    RCC->CR |= RCC_CR_PLLON;                      // Enabling PLL
    while(!(RCC->CR & RCC_CR_PLLRDY));            // Wait until PLL gets locked

    RCC->CFGR |= RCC_CFGR_SW_PLL;                 // Set PLL as main clock
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait until main clock source is switched
    RCC->CR &= ~(RCC_CR_HSION);                   // Disabling internal oscillator
    while(RCC->CR & RCC_CR_HSIRDY);               // Wait until HSI is disabled

    // Enable Cycle Counter, which is used as a steady clock and also for delays
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    ITM->LAR = 0xc5acce55;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
}