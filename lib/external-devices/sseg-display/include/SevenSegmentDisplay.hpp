#ifndef SEVENSEGMENTDISPLAY_HPP
#define SEVENSEGMENTDISPLAY_HPP

#include <cstdint>
#include <STM32F4x1/Gpio.hpp>

namespace Devices
{

using namespace Stm32;

template < typename PinA
         , typename PinB
         , typename PinC
         , typename PinD
         , typename PinE
         , typename PinF
         , typename PinG
         , typename PinCommonAnodeTens
         , typename PinCommonAnodeOnes > class SevenSegmentDisplay
{
  public:

    SevenSegmentDisplay()
    {
      pinA.setMode(GpioMode::Output);
      pinB.setMode(GpioMode::Output);
      pinC.setMode(GpioMode::Output);
      pinD.setMode(GpioMode::Output);
      pinE.setMode(GpioMode::Output);
      pinF.setMode(GpioMode::Output);
      pinG.setMode(GpioMode::Output);
      pinCommonAnodeTens.setMode(GpioMode::Output);
      pinCommonAnodeOnes.setMode(GpioMode::Output);

      pinA.setLevel(segOff);
      pinB.setLevel(segOff);
      pinC.setLevel(segOff);
      pinD.setLevel(segOff);
      pinE.setLevel(segOff);
      pinF.setLevel(segOff);
      pinG.setLevel(segOff);
      pinCommonAnodeTens.setLevel(segOff); // PNP transistor OFF
      pinCommonAnodeOnes.setLevel(segOff); // PNP transistor OFF 
    }

    void setNumber(const uint8_t dist)
    {
      if(dist > 99) return;

      tens = dist / 10;
      ones = dist % 10;
    }

    void setSegments(uint8_t segments)
    {
      // format: 0gfedcba
      pinA.setLevel((segments & 0x01) ? segOn : segOff);
      pinB.setLevel((segments & 0x02)? segOn : segOff);
      pinC.setLevel((segments & 0x04)? segOn : segOff);
      pinD.setLevel((segments & 0x08)? segOn : segOff);
      pinE.setLevel((segments & 0x10)? segOn : segOff); 
      pinF.setLevel((segments & 0x20)? segOn : segOff);
      pinG.setLevel((segments & 0x40)? segOn : segOff);
    }

    void update()
    {
      // format: 0gfedcba
      static constexpr uint8_t ssegMap[10] = {
        0b00111111, // 0
        0b00000110, // 1
        0b01011011, // 2
        0b01001111, // 3
        0b01100110, // 4
        0b01101101, // 5
        0b01111101, // 6
        0b00000111, // 7
        0b01111111, // 8
        0b01101111  // 9
      };
      if(onesOn)
      {
        pinCommonAnodeTens.setLevel(true);  //< turn tens digit off
        setSegments(ssegMap[ones]);
        pinCommonAnodeOnes.setLevel(false);  //< turn ones digit on
      }
      else
      {
        pinCommonAnodeOnes.setLevel(true);  //< turn ones digit off
        setSegments(ssegMap[tens]);
        pinCommonAnodeTens.setLevel(false);  //< turn tens digit on
      }
      onesOn = !onesOn;
    }


  private:
    uint8_t tens = 0, ones = 0;
    bool onesOn = true;
    PinA pinA;
    PinB pinB;
    PinC pinC;
    PinD pinD;
    PinE pinE;
    PinF pinF;
    PinG pinG;
    PinCommonAnodeTens pinCommonAnodeTens;
    PinCommonAnodeOnes pinCommonAnodeOnes;

    static constexpr bool segOn = false;
    static constexpr bool segOff = true;
};
}

#endif // SEVENSEGMENTDISPLAY_HPP
