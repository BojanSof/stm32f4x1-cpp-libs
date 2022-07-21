#ifndef STM32_CLOCK_HPP
#define STM32_CLOCK_HPP

#include <cstdint>

namespace Stm32
{
  inline constexpr uint32_t CoreFrequency = 84000000; // Hz
  void deviceInit();
}

#endif //STM32_CLOCK_HPP