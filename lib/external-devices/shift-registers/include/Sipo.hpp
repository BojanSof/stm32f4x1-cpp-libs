#ifndef SIPO_HPP
#define SIPO_HPP

#include <cstdint>
#include <STM32F4x1/Gpio.hpp>

namespace Devices{

using namespace Stm32;

template <
    typename SerialDataInput
  , typename ShiftClock
  , typename LatchClock
  , typename OutputEnable
  , typename Reset > class SipoRegister
{
  public:
    SipoRegister()
    {
      outputEnable_.setMode(GpioMode::Output);
      latchClock_.setMode(GpioMode::Output);
      serialDataInput_.setMode(GpioMode::Output);
      shiftClock_.setMode(GpioMode::Output);
      reset_.setMode(GpioMode::Output);

      reset_.setLevel(true);
      outputEnable_.setLevel(false);
    }

  void loadStates(const uint8_t dataIn)
  { 
    auto delay = [](const size_t delayCount) {
      for(size_t i = 0; i < delayCount; ++i);
    };
    static constexpr size_t delayCount = 1000;  ///@todo temporary, will be changed with timer delay
    for(uint8_t iData = 0; iData < regSize; ++iData)
    {
      serialDataInput_.setLevel((dataIn >> iData) & 0x01);
      shiftClock_.setLevel(false);
      delay(delayCount);
      shiftClock_.setLevel(true);
      delay(delayCount);
    }
    
    latchClock_.setLevel(false);
    delay(delayCount);
    latchClock_.setLevel(true);
    delay(delayCount);
  }

  void reset()
  {
    reset_.setLevel(false);
    auto delay = [](const size_t delayCount) {
      for(size_t i = 0; i < delayCount; ++i);
    };
    delay(1000);
    reset_.setLevel(true);
  }
   

  private:
    OutputEnable outputEnable_;
    LatchClock latchClock_;
    SerialDataInput serialDataInput_;
    ShiftClock shiftClock_;
    Reset reset_;

    static constexpr size_t regSize = 8;
};

}

#endif // BUTTONS_HPP
