#ifndef PISO_HPP
#define PISO_HPP

#include <cstdint>
#include <cstddef>

#include <Gpio.hpp>

namespace Devices{

using namespace Stm32;

template <
    typename ShiftPin
  , typename ClockPin
  , typename OutputPin > class PisoRegister
{
  public:
    PisoRegister()
    {
      shiftPin_.setMode(GpioMode::Output);
      clockPin_.setMode(GpioMode::Output);
      outputPin_.setMode(GpioMode::Input);

      shiftPin_.setLevel(false);
      clockPin_.setLevel(false);
      outputPin_.setLevel(false);
    }

    uint8_t readStates()
    {
      auto delay = [](const size_t delayCount) {
        for(size_t i = 0; i < delayCount; ++i);
      };
      static constexpr size_t delayCount = 1000;  ///@todo temporary, will be changed with timer delay
      shiftPin_.setLevel(true);   //< shift mode
      delay(delayCount);
      uint8_t data = 0;
      for(uint8_t iData = 0; iData < regSize; ++iData)
      {
        bool state = outputPin_.getState();
        data |= state << iData;
        clockPin_.setLevel(false);
        delay(delayCount);
        clockPin_.setLevel(true);
        delay(delayCount);
      }
      shiftPin_.setLevel(false);  //< load mode
      delay(delayCount);
      return data;    
    }

  private:
    ShiftPin shiftPin_;
    ClockPin clockPin_;
    OutputPin outputPin_;
    static constexpr uint8_t regSize = 8;
};

}

#endif // PISO_HPP
