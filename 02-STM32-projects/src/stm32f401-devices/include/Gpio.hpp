#ifndef STM32F401_GPIO_HPP
#define STM32F401_GPIO_HPP

#include <stm32f4xx.h>
#include <cstdint>

namespace Stm32
{

enum class Port
{
  A,
  B,
  C,
  H
};

enum class GpioMode : uint8_t
{
  Input     = 0b00,
  Output    = 0b01,
  Alternate = 0b10,
  Analog    = 0b11
};

enum class GpioSpeed : uint8_t
{
  Low      = 0b00,
  Medium   = 0b01,
  High     = 0b10,
  VeryHigh = 0b11
};

enum class GpioOutputType : uint8_t
{
  PushPull  = 0,
  OpenDrain = 1
};

enum class GpioPullType : uint8_t
{
  None     = 0b00,
  PullUp   = 0b01,
  PullDown = 0b10,
  Reserved = 0b11
};

enum class GpioAlternateFunction : uint8_t
{
  Af0   = 0x0,
  Af1   = 0x1,
  Af2   = 0x2,
  Af3   = 0x3,
  Af4   = 0x4,
  Af5   = 0x5,
  Af6   = 0x6,
  Af7   = 0x7,
  Af8   = 0x8,
  Af9   = 0x9,
  Af10  = 0xA,
  Af11  = 0xB,
  Af12  = 0xC,
  Af13  = 0xD,
  Af14  = 0xE,
  Af15  = 0xF
};

template <Port port, uint8_t pin> class Gpio
{
  public:
    Gpio()
    {
      static_assert(port == Port::A || port == Port::B ||
                    port == Port::C || port == Port::H,
                    "The specified port is not defined");
      // static_assert(port == PORT::A && (pin == 0 || pin == 1 ||
      //               pin == 2 || pin == 3 || pin == 4 || pin == 5 ||
      //               pin == 6 || pin == 7 || pin == 8 || pin == 9 ||
      //               pin == 10 || pin == 11 || pin == 12 || pin = 13 ||
      //               pin == 14 || pin == 15),
      //               "Invalid pin number");
      // static_assert(port == PORT::B & (pin == 0 || pin == 1 ||
      //               pin == 2 || pin ==3 || pin == 4 || pin == 5 ||
      //               pin == 6 || pin == 7 || pin == 8 || pin == 9 ||
      //               pin == 10 || pin == 12 || pin = 13 ||
      //               pin == 14 || pin == 15),
      //               "Invalid pin number");
      // static_assert(port == PORT::c & (pin == 13 || pin == 14 || pin == 15),
      //               "Invalid pin number");
      // static_assert(port == PORT::H & (pin == 0 || pin == 1), 
      //               "Invalid pin number");
      
      // clock for the port
      if constexpr(port == Port::A)
      {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
      }
      else if constexpr(port == Port::B)
      {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
      }
      else if constexpr(port == Port::C)
      {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
      }
      else if constexpr(port == Port::H)
      {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
      }
    }

    void setMode(const GpioMode& mode)
    {
      gpioInstance_->MODER &= ~(0x3 << (2*pin));
      gpioInstance_->MODER |= static_cast<uint8_t>(mode) << (2*pin);
    }

    void setLevel(const bool lvl)
    {
      gpioInstance_->BSRR = 1 << (pin + (lvl ? 0 : 16));
    }

    bool getState()
    {
      return (gpioInstance_->IDR & (1 << pin));
    }

    void setSpeed(const GpioSpeed& speed)
    {
      gpioInstance_->OSPEEDR &= ~(0x3 << (2*pin));
      gpioInstance_->OSPEEDR |= static_cast<uint8_t>(speed) << (2*pin);
    }

    void setOutputType(const GpioOutputType& output)
    {
      gpioInstance_->OTYPER |= static_cast<uint8_t>(output) << pin;
    }

    void setPullType(const GpioPullType& pull)
    {
      gpioInstance_->PUPDR &= ~(0x3 << (2*pin));
      gpioInstance_->PUPDR |= static_cast<uint8_t>(pull) << (2*pin);
    }

    void setAlternateFunction(const GpioAlternateFunction &af)
    {
      setMode(GpioMode::Alternate);
      if constexpr(pin < 8)
      {
        gpioInstance_->AFR[0] |= static_cast<uint8_t>(af) << (4*pin);
      }
      else
      {
        gpioInstance_->AFR[1] |= static_cast<uint8_t>(af) << (4*pin);
      }
    }
  private:
    constexpr static GPIO_TypeDef * getGpioInstance()
    {
      switch(port)
      {
        case Port::A: return GPIOA;
        break;
        case Port::B: return GPIOB;
        break;
        case Port::C: return GPIOC;
        break;
        case Port::H: return GPIOH;
        break;
        default: return nullptr;
        break;
      }
    }
    GPIO_TypeDef * const gpioInstance_ = getGpioInstance();
};

}

#endif //STM32F401_GPIO_HPP