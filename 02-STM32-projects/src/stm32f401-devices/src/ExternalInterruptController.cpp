#include "ExternalInterruptController.hpp"

namespace Stm32
{
  void checkPinInterrupt(const uint8_t pinNumber)
  {
    if((EXTI->PR & (1 << pinNumber)))
    {
      if(ExternalInterruptController::callbacks_[pinNumber])
      {
        ExternalInterruptController::callbacks_[pinNumber]();
      }
      EXTI->PR |= 1 << pinNumber;
    }
  };
}

extern "C"
{
  void EXTI0_IRQHandler()
  {
    using namespace Stm32;
    checkPinInterrupt(0);
  }

  void EXTI1_IRQHandler()
  {
    using namespace Stm32;
    checkPinInterrupt(1);
  }

  void EXTI2_IRQHandler()
  {
    using namespace Stm32;
    checkPinInterrupt(2);
  }

  void EXTI3_IRQHandler()
  {
    using namespace Stm32;
    checkPinInterrupt(3);
  }

  void EXTI4_IRQHandler()
  {
    using namespace Stm32;
    checkPinInterrupt(4);
  }

  void EXTI9_5_IRQHandler()
  {
    using namespace Stm32;
    for(uint8_t i = 5; i <= 9; ++i)
    {
      checkPinInterrupt(i);
    }
  }

  void EXTI15_10_IRQHandler()
  {
    using namespace Stm32;
    for(uint8_t i = 10; i <= 15; ++i)
    {
      checkPinInterrupt(i);
    }
  }
}