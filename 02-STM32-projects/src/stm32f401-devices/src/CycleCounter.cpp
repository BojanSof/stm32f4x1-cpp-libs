#include <stm32f4xx.h>

#include "CycleCounter.hpp"


namespace Stm32
{
  CycleCounter::CycleCounter()
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }

  CycleCounter& CycleCounter::getInstance()
  {
    static CycleCounter cycleCounter;
    return cycleCounter;
  }

  void CycleCounter::start()
  {
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  void CycleCounter::stop()
  {
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  }

  uint32_t CycleCounter::getCount()
  {
    return DWT->CYCCNT;
  }

  void CycleCounter::delayUs(const uint32_t us)
  {
    const uint32_t startCount = DWT->CYCCNT;
    const uint32_t delayTicks = usToTicks(us);
    while(DWT->CYCCNT - startCount < delayTicks);  //< overflow once handled by unsigned difference 
  }

  void CycleCounter::delayMs(const uint32_t ms)
  {
    const uint32_t startCount = DWT->CYCCNT;
    const uint32_t delayTicks = msToTicks(ms);
    while(DWT->CYCCNT - startCount < delayTicks);  //< overflow once handled by unsigned difference 
  }

}