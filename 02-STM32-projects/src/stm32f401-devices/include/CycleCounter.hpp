#ifndef STM32_CYCLE_COUNTER
#define STM32_CYCLE_COUNTER

#include <cstdint>
#include "Clock.hpp"

namespace Stm32
{
  class CycleCounter
  {
    public:
      static CycleCounter& getInstance();

      void start();
      void stop();
      uint32_t getCount();

      void delayUs(const uint32_t us);
      void delayMs(const uint32_t ms);

      constexpr uint32_t ticksToUs(const uint32_t ticks)
      {
        return ticks/(CoreFrequency/1000000U);
      }

      constexpr uint32_t usToTicks(const uint32_t us)
      {
        return us * (CoreFrequency/1000000U);
      }

      constexpr uint32_t ticksToMs(const uint32_t ticks)
      {
        return ticks/(CoreFrequency/1000U);
      }

      constexpr uint32_t msToTicks(const uint32_t ms)
      {
        return ms * (CoreFrequency/1000U);
      }

    private:
      CycleCounter();
      CycleCounter(const CycleCounter&) = delete;  // copy constructor
      CycleCounter(const CycleCounter&&) = delete; // move constructor
      void operator=(const CycleCounter&) = delete;
      void operator=(const CycleCounter&&) = delete;

  };
}

#endif //STM32_CYCLE_COUNTER