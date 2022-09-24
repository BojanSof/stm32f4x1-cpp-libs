#ifndef STM32_GPIO_TYPES_HPP
#define STM32_GPIO_TYPES_HPP

#include <stm32f4xx.h>

#include <cstdint>

namespace Stm32
{
  /**
   * Possible GPIO ports for STM32F401CCU6,
   * UQFN48 package
   * 
   */
  enum class Port : uint8_t
  {
    A = 0,
    B = 1,
    C = 2,
    H = 7
  };

  enum class GpioMode : uint8_t
  {
    Input = 0b00,
    Output = 0b01,
    Alternate = 0b10,
    Analog = 0b11
  };

  enum class GpioSpeed : uint8_t
  {
    Low = 0b00,
    Medium = 0b01,
    High = 0b10,
    VeryHigh = 0b11
  };

  enum class GpioOutputType : uint8_t
  {
    PushPull = 0,
    OpenDrain = 1
  };

  enum class GpioPullType : uint8_t
  {
    None = 0b00,
    PullUp = 0b01,
    PullDown = 0b10,
    Reserved = 0b11
  };

  enum class GpioExternalInterruptEdge : uint8_t
  {
    Rising  = 0,
    Falling = 1,
    Both    = 2
  };

  enum class GpioAlternateFunctionNumber : uint8_t
  {
    Af0 = 0x0,
    Af1 = 0x1,
    Af2 = 0x2,
    Af3 = 0x3,
    Af4 = 0x4,
    Af5 = 0x5,
    Af6 = 0x6,
    Af7 = 0x7,
    Af8 = 0x8,
    Af9 = 0x9,
    Af10 = 0xA,
    Af11 = 0xB,
    Af12 = 0xC,
    Af13 = 0xD,
    Af14 = 0xE,
    Af15 = 0xF
  };

  constexpr GPIO_TypeDef* getGpioInstance(const Port& port)
  {
    switch (port)
    {
    case Port::A:
      return GPIOA;
      break;
    case Port::B:
      return GPIOB;
      break;
    case Port::C:
      return GPIOC;
      break;
    case Port::H:
      return GPIOH;
      break;
    default:
      return nullptr;
      break;
    }
  }
}

#endif //STM32_GPIO_TYPES_HPP