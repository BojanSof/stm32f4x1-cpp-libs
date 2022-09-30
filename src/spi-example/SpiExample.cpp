#include <STM32F4x1/SPI.hpp>
#include <STM32F4x1/Gpio.hpp>
#include <STM32F4x1/Clock.hpp>
#include <STM32F4x1/CycleCounter.hpp>

int main()
{
  // ensure Arduino UNO runs the SPI slave sketch included in this file's directory
  using namespace Stm32;
  deviceInit();
  
  using MosiPin = Pins::PB15;
  using MisoPin = Pins::PB14;
  using SckPin = Pins::PB13;
  using SsPin = Pins::PB12;
  
  using ConfigPins = SpiPins<MosiPin, MisoPin, SckPin, SsPin>;

  using Config = SpiConfig<
      1000000ul        //< SCK freq
    , SpiMode::Mode0   //< SPI mode
    , false            //< 8-bit frame size
    , false            //< send MSB first
    , ConfigPins       //< SPI pins
    , false             //< hardware SS control
  >;
  // configure the spi
  auto& spi = SPI<2>::getInstance();
  spi.configure<Config>();
  
  static constexpr uint8_t dataToSend = 0xAB;
  uint8_t dataRead = 0;
  while(true)
  {
    spi.transfer(reinterpret_cast<std::byte*>(&dataRead), reinterpret_cast<const std::byte*>(&dataToSend), sizeof(dataToSend));
    using namespace std::chrono_literals;
    CycleCounter::delay(500ms);
  }
}