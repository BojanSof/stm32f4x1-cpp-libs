#ifndef TOUCH_HPP
#define TOUCH_HPP

#include <algorithm>
#include <cstdint>
#include <type_traits>
#include <optional>

#include <SPI.hpp>

namespace Devices
{
  template<
      typename CoordinatesT
    , typename = typename std::enable_if_t<std::is_arithmetic_v<CoordinatesT>>
  >
  struct TouchCoordinates
  {
    CoordinatesT x, y, z;
  };

  template<
      typename SpiInterfaceT
    , typename MosiPinT
    , typename MisoPinT
    , typename SckPinT
    , typename SsPinT
  >
  class Touch
  {
    public:
      std::optional<TouchCoordinates<uint16_t>> readRawData()
      {
        using namespace Stm32;
        auto& spi = SpiInterfaceT::getInstance();
        // configure SPI
        using ConfigPins = SpiPins<MosiPinT, MisoPinT, SckPinT, SsPinT>;
        using Config = SpiConfig<
            1000000ul        //< SCK freq
          , SpiMode::Mode0   //< SPI mode
          , false            //< 8-bit frame size
          , false            //< send MSB first
          , ConfigPins       //< SPI pins
          , true             //< software SS control
        >;
        spi.template configure<Config>();

        uint8_t readData[3]{};
        uint8_t writeData[3]{};
        // read Z1
        writeData[0] = ReadZ1Cmd;
        spi.transfer(reinterpret_cast<std::byte*>(readData), reinterpret_cast<const std::byte*>(writeData), 3);
        uint16_t z1 = (readData[1] << 5) | (readData[2] >> 3);
        // read Z2
        writeData[0] = ReadZ2Cmd;
        spi.transfer(reinterpret_cast<std::byte*>(readData), reinterpret_cast<const std::byte*>(writeData), 3);
        uint16_t z2 = (readData[1] << 5) | (readData[2] >> 3);
        const uint16_t z = 4095 - z2 + z1;
        if(z < zThreshold) return std::nullopt;
        // N reads of X
        // N reads of Y
        static constexpr size_t NumReads = 16;
        uint16_t xRaw[NumReads]{}, yRaw[NumReads]{};
        for(size_t iRead = 0; iRead < NumReads; ++iRead)
        {
          // read X
          writeData[0] = ReadXCmd;
          spi.transfer(reinterpret_cast<std::byte*>(readData), reinterpret_cast<const std::byte*>(writeData), 3);
          xRaw[iRead] = (readData[1] << 5) | (readData[2] >> 3);
          // read Y
          writeData[0] = ReadYCmd;
          spi.transfer(reinterpret_cast<std::byte*>(readData), reinterpret_cast<const std::byte*>(writeData), 3);
          yRaw[iRead] = (readData[1] << 5) | (readData[2] >> 3);
        }
        // sort the arrays
        std::sort(std::begin(xRaw), std::end(xRaw));
        std::sort(std::begin(yRaw), std::end(yRaw));
        // average the samples without min and max
        uint32_t xSum = 0;
        uint32_t ySum = 0;
        for(size_t iRead = 1; iRead < NumReads - 1; ++iRead)
        {
          xSum += xRaw[iRead];
          ySum += yRaw[iRead];
        }
        const uint16_t x = xSum/(NumReads - 2);
        const uint16_t y = ySum/(NumReads - 2);

        return TouchCoordinates<uint16_t>{x, y, z};
      }

    private:
      static constexpr uint8_t ReadZ1Cmd  = 0xB0;
      static constexpr uint8_t ReadZ2Cmd  = 0xC0;
      static constexpr uint8_t ReadXCmd   = 0xD0;
      static constexpr uint8_t ReadYCmd   = 0x90;

      static constexpr uint16_t zThreshold = 300;
  };
}

#endif //TOUCH_HPP