#ifndef STM32F401_GPIO_HPP
#define STM32F401_GPIO_HPP

#include "GpioTypes.hpp"
#include "GpioAlternateFunctions.hpp"
#include "ExternalInterruptController.hpp"

#include <stm32f4xx.h>
#include <bitset>
#include <cstdint>
#include <functional>
#include <tuple>


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

    /**
     * @brief Enable external interrupt for the pin
     * 
     * @tparam Edge The edge on which the interrupt is triggered
     * @param callback The callback to execute when interrupt fires
     */
    template<GpioExternalInterruptEdge Edge>
    void enableInterrupt(const std::function<void()>& callback)
    {
      auto& exti = ExternalInterruptController::getInstance();
      exti.enableInterrupt<Gpio<port, pin>, Edge>(callback);
    }

    /**
     * Disable external interrupt for the pin
     * 
     */
    void disableInterrupt()
    {
      auto& exti = ExternalInterruptController::getInstance();
      exti.disableInterrupt<Gpio<port, pin>>();
    }

    /**
     * @brief Set the mode for the pin, i.e. input, output, etc.
     * 
     * @param mode The mode for the pin
     */
    void setMode(const GpioMode& mode)
    {
      gpioInstance_->MODER &= ~(0x3 << (2 * pin));
      gpioInstance_->MODER |= static_cast<uint8_t>(mode) << (2 * pin);
    }

    /**
     * @brief Set the level of the pin
     * 
     * @param lvl True to set pin high, false to set pin low
     */
    void setLevel(const bool lvl)
    {
      gpioInstance_->BSRR = 1 << (pin + (lvl ? 0 : 16));
    }

    /**
     * @brief Read the state of the pin
     * 
     * @return true The pin is high
     * @return false The pin is low
     */
    bool getState()
    {
      return (gpioInstance_->IDR & (1 << pin));
    }

    /**
     * @brief Set the speed of the pin (controls the slew rate)
     * 
     * @param speed The speed of the pin
     */
    void setSpeed(const GpioSpeed& speed)
    {
      gpioInstance_->OSPEEDR &= ~(0x3 << (2 * pin));
      gpioInstance_->OSPEEDR |= static_cast<uint8_t>(speed) << (2 * pin);
    }

    /**
     * @brief Set the Output Type of the pin, i.e. push-pull or open-drain
     * 
     * @param output The output type of the pin
     */
    void setOutputType(const GpioOutputType& output)
    {
      if(output == GpioOutputType::PushPull)
      {
        gpioInstance_->OTYPER &= ~(static_cast<uint8_t>(output) << pin);
      }
      else
      {
        gpioInstance_->OTYPER |= static_cast<uint8_t>(output) << pin;
      }
    }

    /**
     * @brief Set the internal pull up/down resistors
     * for the pin
     * 
     * @param pull The type of the internal resistor(s) to use
     */
    void setPullType(const GpioPullType& pull)
    {
      gpioInstance_->PUPDR &= ~(0x3 << (2 * pin));
      gpioInstance_->PUPDR |= static_cast<uint8_t>(pull) << (2 * pin);
    }

    /**
     * @brief Set the alternate function of the pin
     * based on the alternate function number
     * 
     * @param af The alternate function number
     */
    void setAlternateFunction(const GpioAlternateFunctionNumber& af)
    {
      setMode(GpioMode::Alternate);
      if constexpr (pin < 8)
      {
        gpioInstance_->AFR[0] |= static_cast<uint8_t>(af) << (4 * pin);
      }
      else
      {
        gpioInstance_->AFR[1] |= static_cast<uint8_t>(af) << (4 * (pin - 8));
      }
    }

    /**
     * @brief Set the alternate function for the pin
     * based on its available alternate functions
     * 
     * @param af The alternate function to set
     */
    void setAlternateFunction(const typename AlternateFunctions::Type af)
    {
      setAlternateFunction(static_cast<GpioAlternateFunctionNumber>(static_cast<uint16_t>(af) >> 8));
    }

    /**
     * @brief Check if pin has the specified alternate function
     * 
     * @param af The ID of the alternate function
     * @return true The pin has the specified alternate function
     * @return false The pin does not have the specified alternate function
     */
    constexpr bool checkAlternateFunction(const GpioAlternateFunctionId af)
    {
      return gpioCheckAlternateFunction<port, pin>(af);
    }

  private:
    GPIO_TypeDef *const gpioInstance_ = getGpioInstance(port);
  };

  // Function that work on multiple pins
  namespace Pins
  {
    /**
     * @brief Check if the pins are part of the same port
     * 
     * @tparam Pins The pins to check
     * @return true Pins are part of the same port
     * @return false Pins are not part of the same port
     */
    template<typename... Pins>
    constexpr bool gpioSamePort()
    {
      return ((Pins::pinPort == (std::tuple_element_t<0, std::tuple<Pins...>>::pinPort)) && ... );
    }

    /**
     * @brief Set the mode for multiple pins
     * 
     * @tparam Pins The pins to change the mode for
     * @param mode The mode of the pins
     */
    template<typename... Pins>
    void setMode(const GpioMode& mode)
    {
      static_assert(sizeof...(Pins) > 0, "Number of specified pins must be positive");
      if constexpr(gpioSamePort<Pins...>())
      {
        using FirstPin = std::tuple_element_t<0, std::tuple<Pins...>>;
        auto port = getGpioInstance(FirstPin::pinPort);
        uint32_t moder = port->MODER;
        ([&]{
          moder &= ~(0x3 << (2 * Pins::pinNumber));
          moder |= static_cast<uint8_t>(mode) << (2 * Pins::pinNumber);
        }(), ...);
        port->MODER = moder;
      }
      else
      {
        ([&]{
          Pins{}.setMode(mode);
        }(), ...);
      }
    }

    /**
     * @brief Set the speed for multiple pins
     * 
     * @tparam Pins The pins to change the speed for
     * @param speed The speed for the pins
     */
    template<typename... Pins>
    void setSpeed(const GpioSpeed& speed)
    {
      static_assert(sizeof...(Pins) > 0, "Number of specified pins must be positive");
      if constexpr(gpioSamePort<Pins...>())
      {
        using FirstPin = std::tuple_element_t<0, std::tuple<Pins...>>;
        auto port = getGpioInstance(FirstPin::pinPort);
        uint32_t ospeedr = port->OSPEEDR;
        ([&]{
          ospeedr &= ~(0x3 << (2 * Pins::pinNumber));
          ospeedr |= static_cast<uint8_t>(speed) << (2 * Pins::pinNumber);
        }(), ...);
        port->OSPEEDR = ospeedr;
      }
      else
      {
        ([&]{
          Pins{}.setSpeed(speed);
        }(), ...);
      }
    }

    /**
     * @brief Set the output type for multiple pins
     * 
     * @tparam Pins The pins to change the output type for
     * @param output The output type for the pins
     */
    template<typename... Pins>
    void setOutputType(const GpioOutputType& output)
    {
      static_assert(sizeof...(Pins) > 0, "Number of specified pins must be positive");
      if constexpr(gpioSamePort<Pins...>())
      {
        using FirstPin = std::tuple_element_t<0, std::tuple<Pins...>>;
        auto port = getGpioInstance(FirstPin::pinPort);
        uint32_t otyper = port->OTYPER;
        ([&]{
          if(output == GpioOutputType::PushPull)
          {
            otyper &= ~(static_cast<uint8_t>(output) << Pins::pinNumber);
          }
          else
          {
            otyper |= static_cast<uint8_t>(output) << Pins::pinNumber;
          }
        }(), ...);
        port->OTYPER = otyper;
      }
      else
      {
        ([&]{
          Pins{}.setOutputType(output);
        }(), ...);
      }
    }

    /**
     * @brief Set the pull type for multiple pins
     * 
     * @tparam Pins The pins to change the pull type for
     * @param pull The pull type
     */
    template<typename... Pins>
    void setPullType(const GpioPullType& pull)
    {
      static_assert(sizeof...(Pins) > 0, "Number of specified pins must be positive");
      if constexpr(gpioSamePort<Pins...>())
      {
        using FirstPin = std::tuple_element_t<0, std::tuple<Pins...>>;
        auto port = getGpioInstance(FirstPin::pinPort);
        uint32_t pupdr = port->PUPDR;
        ([&]{
          pupdr &= ~(0x3 << (2 * Pins::pinNumber));
          pupdr |= static_cast<uint8_t>(pull) << (2 * Pins::pinNumber);
        }(), ...);
        port->PUPDR = pupdr;
      }
      else
      {
        ([&]{
          Pins{}.setPullType(pull);
        }(), ...);
      }
    }

    /**
     * @brief Set the level of the specified pins
     * 
     * @tparam Pins The pins to change the levels for
     * @param levels The levels for the pins
     */
    
    template<typename... Pins>
    void setLevel(const uint32_t levels)
    {
      static_assert(sizeof...(Pins) > 0, "Number of specified pins must be positive");
      if constexpr(gpioSamePort<Pins...>())
      {
        using FirstPin = std::tuple_element_t<0, std::tuple<Pins...>>;
        auto port = getGpioInstance(FirstPin::pinPort);
        uint32_t bsrr = 0;
        size_t iLevel = 0;
        ([&]{
          bsrr |= 1 << (Pins::pinNumber + ((levels & (1 << iLevel++)) ? 0 : 16));
        }(), ...);
        port->BSRR = bsrr;
      }
      else
      {
        size_t iLevel = 0;
        ([&]{
          Pins{}.setLevel(levels & (1 << iLevel++));
        }(), ...);
      }
    }
  }

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