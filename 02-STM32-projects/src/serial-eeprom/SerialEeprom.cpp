#include <Gpio.hpp>
#include <Clock.hpp>
#include <CycleCounter.hpp>
#include <I2C.hpp>

int main()
{
  using namespace Stm32;
  
  deviceInit();
  auto& i2cInterface = I2C<1>::getInstance();
  using eepromI2Cconfig = I2Cconfig<
    100000U,    // frequency
    0x50,       // address
    Pins::PB7,  // SDA pin
    Pins::PB6   // SCL pin
  >;
  i2cInterface.configure<eepromI2Cconfig>();
  while(true)
  {
    // write byte on address 0x0000
    const uint8_t arrWrite[] = {0x00, 0x00, 0xab};
    i2cInterface.write(reinterpret_cast<const std::byte*>(arrWrite), sizeof(arrWrite));
    // read byte from address 0x0000
    using namespace std::chrono_literals;
    CycleCounter::delay(10ms);
    uint8_t data = 0;
    const uint8_t address[] = {0x00, 0x00};
    i2cInterface.write(reinterpret_cast<const std::byte*>(address), sizeof(address));
    i2cInterface.read(reinterpret_cast<std::byte*>(&data), sizeof(data));
  }

  return 0;
}
