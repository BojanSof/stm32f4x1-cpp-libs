#include <stm32f4xx.h>
#include <functional>
#include "GpioTypes.hpp"

namespace Stm32
{
  class ExternalInterruptController
  {
    public:
      static ExternalInterruptController& getInstance()
      {
        static ExternalInterruptController instance;
        return instance;
      }

      template<typename Pin, GpioExternalInterruptEdge Edge>
      void enableInterrupt(const std::function<void()>& callback)
      {
        constexpr auto port = static_cast<uint8_t>(Pin::pinPort);
        constexpr auto pinNumber = Pin::pinNumber;
        SYSCFG->EXTICR[pinNumber/4] &= ~(0xF << (4 * (pinNumber % 4)));
        SYSCFG->EXTICR[pinNumber/4] |= port << (4 * (pinNumber % 4));

        if constexpr(Edge == GpioExternalInterruptEdge::Rising
                    || Edge == GpioExternalInterruptEdge::Both)
        {
          EXTI->RTSR |= 1 << pinNumber;
        }
        else if constexpr(Edge == GpioExternalInterruptEdge::Falling
                          || Edge == GpioExternalInterruptEdge::Both)
        {
          EXTI->FTSR |= 1 << pinNumber;
        }

        EXTI->IMR |= 1 << pinNumber;
        ExternalInterruptController::callbacks_[pinNumber] = callback;

        constexpr IRQn_Type extiIRQn = getExtiIRQn(pinNumber);
        if(!NVIC_GetEnableIRQ(extiIRQn))
        {
          NVIC_ClearPendingIRQ(extiIRQn);
          NVIC_EnableIRQ(extiIRQn);
        }
      }

      template<typename Pin>
      void disableInterrupt()
      {
        constexpr auto port = static_cast<uint8_t>(Pin::pinPort);
        constexpr auto pinNumber = Pin::pinNumber;
        SYSCFG->EXTICR[pinNumber/4] &= ~(0xF << (4 * (pinNumber % 4)));
        
        EXTI->IMR &= ~(1 << pinNumber);
        ExternalInterruptController::callbacks_[pinNumber] = nullptr;

        bool otherIrqActive = false;
        if constexpr (pinNumber >= 5 && pinNumber <= 9)
        {
          otherIrqActive = EXTI->IMR & (0x1F << 5);
        }
        else if constexpr (pinNumber >= 10 && pinNumber <= 15)
        {
          otherIrqActive = EXTI->IMR & (0x3F << 10);
        }
        if(!otherIrqActive)
        {
          constexpr IRQn_Type extiIRQn = getExtiIRQn(pinNumber);
          NVIC_DisableIRQ(extiIRQn);
        }
      }

      template<typename Pin>
      void generateInterrupt()
      {
        constexpr auto pinNumber = Pin::pinNumber;
        EXTI->SWIER |= 1 << pinNumber;
      }

      template<typename Pin>
      bool getInterruptStatus()
      {
        constexpr auto pinNumber = Pin::pinNumber;
        return (EXTI->PR & (1 << pinNumber));
      }
      
      constexpr IRQn_Type getExtiIRQn(const uint8_t pinNumber)
      {
        IRQn_Type extiIRQn = static_cast<IRQn_Type>(-100);
        if (pinNumber == 0)
        {
          extiIRQn = EXTI0_IRQn;
        }
        else if (pinNumber == 1)
        {
          extiIRQn = EXTI1_IRQn;
        }
        else if (pinNumber == 2)
        {
          extiIRQn = EXTI2_IRQn;
        }
        else if (pinNumber == 3)
        {
          extiIRQn = EXTI3_IRQn;
        }
        else if (pinNumber == 4)
        {
          extiIRQn = EXTI4_IRQn;
        }
        else if (pinNumber >= 5 && pinNumber <= 9)
        {
          extiIRQn = EXTI9_5_IRQn;
        }
        else if (pinNumber >= 10 && pinNumber <= 15)
        {
          extiIRQn = EXTI15_10_IRQn;
        }
        return extiIRQn;
      }

    private:
      ExternalInterruptController()
      {
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
      }

      ExternalInterruptController(const ExternalInterruptController &) = delete;
      ExternalInterruptController(ExternalInterruptController &&) = delete;
      void operator=(const ExternalInterruptController &) = delete;
      void operator=(ExternalInterruptController &&) = delete;
    public:
      static constexpr size_t numChannels_ = 16;
      inline static std::function<void()> callbacks_[numChannels_];
  };
}