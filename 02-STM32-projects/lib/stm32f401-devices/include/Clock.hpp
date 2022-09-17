#ifndef STM32_CLOCK_HPP
#define STM32_CLOCK_HPP

#include <cstdint>

namespace Stm32
{
  inline constexpr uint32_t CoreFrequency = 84000000U; // Hz
  inline constexpr uint32_t Pclk1Frequency = 42000000U; // Hz, APB1 clock
  inline constexpr uint32_t Pclk2Frequency = 84000000U; // Hz, APB2 clock
  void deviceInit();
}

#endif //STM32_CLOCK_HPP