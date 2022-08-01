#ifndef STM32_CYCLE_COUNTER
#define STM32_CYCLE_COUNTER

#include <stm32f4xx.h>

#include <cstdint>
#include <chrono>
#include "Clock.hpp"

namespace Stm32
{
  /**
   * Class providing steady clock using
   * Cortex-M4 Cycle Counter timer
   */
  class CycleCounter
  {
    public:
      using period = std::ratio<1, CoreFrequency>;
      using rep = int32_t;
      using duration = std::chrono::duration<rep, period>;
      using time_point = std::chrono::time_point<CycleCounter>;
      static constexpr bool is_steady = true;

      /**
       * @brief Get the current time point, measured from
       * the Cycle Counter 0 value
       * 
       * @return time_point Current Cycle Counter time point
       */
      static time_point now() noexcept
      {
        return time_point{ duration{ DWT->CYCCNT } };
      }

      /**
       * @brief Delay the execution of the code
       * for at least the specified duration
       * 
       * @param dur Time duration for delay
       */
      template< class Rep, class Period >
      static void delay(const std::chrono::duration<Rep, Period>& dur)
      {
        auto t = now();
        while(now() - t < dur());
      }
  };
}

#endif //STM32_CYCLE_COUNTER