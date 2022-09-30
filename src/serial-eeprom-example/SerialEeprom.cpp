#include <STM32F4x1/Gpio.hpp>
#include <STM32F4x1/Clock.hpp>
#include <STM32F4x1/I2C.hpp>

#include <SerialEeprom.hpp>

int main()
{
  using namespace Stm32;
  deviceInit();
  Devices::SerialEeprom<
      64
    , I2C<1>
    , 0x50
    , Pins::PB7
    , Pins::PB6
  > eeprom;
  
  while(true)
  {
    // write byte on address 0x0000
    const uint8_t dataToWrite[] = {0xde, 0xad, 0xbe, 0xef};
    const uint16_t startAddress = 0x0000;
    eeprom.writeBytes(reinterpret_cast<const std::byte*>(dataToWrite), startAddress, sizeof(dataToWrite));
    // read bytes from address 0x0000
    uint8_t readData[4] = {0};
    eeprom.readBytes(reinterpret_cast<std::byte*>(readData), startAddress, sizeof(readData));
  }

  return 0;
}
