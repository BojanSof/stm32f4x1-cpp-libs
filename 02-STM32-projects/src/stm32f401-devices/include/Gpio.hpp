#ifndef STM32F401_GPIO_HPP
#define STM32F401_GPIO_HPP

#include <stm32f4xx.h>
#include <cstdint>

namespace Stm32
{

  /**
   * Possible GPIO ports for STM32F401CCU6,
   * UQFN48 package
   * 
   */
  enum class Port
  {
    A,
    B,
    C,
    H
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

  template <Port port, uint8_t pin>
  struct GpioAlternateFunction
  {
    enum Type : uint8_t
    {
      Default = 0
    } type_;
  };

  template <Port port, uint8_t pin>
  class Gpio
  {
  public:
    using AlternateFunctions = GpioAlternateFunction<port, pin>;
    static constexpr Port pinPort = port;
    static constexpr uint8_t pinNumber = pin;

    Gpio()
    {
      static_assert(port == Port::A || port == Port::B ||
                    port == Port::C || port == Port::H,
                    "The specified port is not defined");

      // clock for the port
      if constexpr (port == Port::A)
      {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
      }
      else if constexpr (port == Port::B)
      {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
      }
      else if constexpr (port == Port::C)
      {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
      }
      else if constexpr (port == Port::H)
      {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
      }
    }

    void setMode(const GpioMode &mode)
    {
      gpioInstance_->MODER &= ~(0x3 << (2 * pin));
      gpioInstance_->MODER |= static_cast<uint8_t>(mode) << (2 * pin);
    }

    void setLevel(const bool lvl)
    {
      gpioInstance_->BSRR = 1 << (pin + (lvl ? 0 : 16));
    }

    bool getState()
    {
      return (gpioInstance_->IDR & (1 << pin));
    }

    void setSpeed(const GpioSpeed &speed)
    {
      gpioInstance_->OSPEEDR &= ~(0x3 << (2 * pin));
      gpioInstance_->OSPEEDR |= static_cast<uint8_t>(speed) << (2 * pin);
    }

    void setOutputType(const GpioOutputType &output)
    {
      gpioInstance_->OTYPER |= static_cast<uint8_t>(output) << pin;
    }

    void setPullType(const GpioPullType &pull)
    {
      gpioInstance_->PUPDR &= ~(0x3 << (2 * pin));
      gpioInstance_->PUPDR |= static_cast<uint8_t>(pull) << (2 * pin);
    }

    void setAlternateFunction(const GpioAlternateFunctionNumber &af)
    {
      setMode(GpioMode::Alternate);
      if constexpr (pin < 8)
      {
        gpioInstance_->AFR[0] |= static_cast<uint8_t>(af) << (4 * pin);
      }
      else
      {
        gpioInstance_->AFR[1] |= static_cast<uint8_t>(af) << (4 * pin);
      }
    }

    void setAlternateFunction(const typename AlternateFunctions::Type af)
    {
      setAlternateFunction(static_cast<GpioAlternateFunctionNumber>(af));
    }

  private:
    constexpr static GPIO_TypeDef *getGpioInstance()
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
    GPIO_TypeDef *const gpioInstance_ = getGpioInstance();
  };

  // Specialize the alternate functions template class for each pin
  template<>
  struct GpioAlternateFunction<Port::A, 0>
  {
    enum Type : uint8_t
    {
      Default = 0,
      TIM2_CH1 = 1,
      TIM5_CH1 = 2,
      USART2_CTS = 7,
      EVENT_OUT = 15
    } type_;
  };
  ///@todo Specialize for other pins

  // Pins on STM32F401CCU6, UQFN48 package
  namespace Pins
  {
    using PA0 = Gpio<Port::A, 0>;
    using PA1 = Gpio<Port::A, 1>;
    using PA2 = Gpio<Port::A, 2>;
    using PA3 = Gpio<Port::A, 3>;
    using PA4 = Gpio<Port::A, 4>;
    using PA5 = Gpio<Port::A, 5>;
    using PA6 = Gpio<Port::A, 6>;
    using PA7 = Gpio<Port::A, 7>;
    using PA8 = Gpio<Port::A, 8>;
    using PA9 = Gpio<Port::A, 9>;
    using PA10 = Gpio<Port::A, 10>;
    using PA11 = Gpio<Port::A, 11>;
    using PA12 = Gpio<Port::A, 12>;
    using PA13 = Gpio<Port::A, 13>;
    using PA14 = Gpio<Port::A, 14>;
    using PA15 = Gpio<Port::A, 15>;

    using PB0 = Gpio<Port::B, 0>;
    using PB1 = Gpio<Port::B, 1>;
    using PB2 = Gpio<Port::B, 2>;
    using PB3 = Gpio<Port::B, 3>;
    using PB4 = Gpio<Port::B, 4>;
    using PB5 = Gpio<Port::B, 5>;
    using PB6 = Gpio<Port::B, 6>;
    using PB7 = Gpio<Port::B, 7>;
    using PB8 = Gpio<Port::B, 8>;
    using PB9 = Gpio<Port::B, 9>;
    using PB10 = Gpio<Port::B, 10>;
    using PB12 = Gpio<Port::B, 12>;
    using PB13 = Gpio<Port::B, 13>;
    using PB14 = Gpio<Port::B, 14>;
    using PB15 = Gpio<Port::B, 15>;

    using PC13 = Gpio<Port::C, 13>;
    using PC14 = Gpio<Port::C, 14>;
    using PC15 = Gpio<Port::C, 15>;

    using PH0 = Gpio<Port::H, 0>;
    using PH1 = Gpio<Port::H, 1>;
  }

}

#endif // STM32F401_GPIO_HPP