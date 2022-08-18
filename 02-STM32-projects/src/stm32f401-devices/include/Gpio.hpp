#ifndef STM32F401_GPIO_HPP
#define STM32F401_GPIO_HPP

#include "GpioTypes.hpp"
#include "GpioAlternateFunctions.hpp"
#include "ExternalInterruptController.hpp"

#include <stm32f4xx.h>
#include <cstdint>
#include <functional>


namespace Stm32
{
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

    template<GpioExternalInterruptEdge Edge>
    void enableInterrupt(const std::function<void>& callback)
    {
      auto& exti = ExternalInterruptController::getInstance();
      exti.enableInterrupt<Gpio<port, pin>, Edge>(callback);
    }

    void disableInterrupt()
    {
      auto& exti = ExternalInterruptController::getInstance();
      exti.disableInterrupt<Gpio<port, pin>>();
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
      setAlternateFunction(static_cast<GpioAlternateFunctionNumber>(static_cast<uint16_t>(af) >> 8));
    }

    constexpr bool checkAlternateFunction(const GpioAlternateFunctionId af)
    {
      return gpioCheckAlternateFunction<port, pin>(af);
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