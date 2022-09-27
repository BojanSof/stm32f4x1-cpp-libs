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
          , true             //< user controls SS
        >;
        spi.template configure<Config>();

        SsPinT slaveSelect;
        uint8_t rawSpiData[2]{};
        uint8_t zerosSpi[2]{0, 0};
        // read Z1
        slaveSelect.setLevel(false);
        spi.write(reinterpret_cast<const std::byte*>(&ReadZ1Cmd), 1);
        spi.transfer(reinterpret_cast<std::byte*>(rawSpiData), reinterpret_cast<const std::byte*>(zerosSpi), 2);
        slaveSelect.setLevel(true);
        uint16_t z1 = (rawSpiData[0] << 4) | (rawSpiData[1] >> 3);
        // read Z2
        slaveSelect.setLevel(false);
        spi.write(reinterpret_cast<const std::byte*>(&ReadZ2Cmd), 1);
        spi.transfer(reinterpret_cast<std::byte*>(rawSpiData), reinterpret_cast<const std::byte*>(zerosSpi), 2);
        slaveSelect.setLevel(true);
        uint16_t z2 = (rawSpiData[0] << 4) | (rawSpiData[1] >> 3);
        const uint16_t z = 4095 - z2 + z1;
        if(z < zThreshold) return std::nullopt;
        // N reads of X
        // N reads of Y
        static constexpr size_t NumReads = 16;
        uint16_t xRaw[NumReads]{}, yRaw[NumReads]{};
        for(size_t iRead = 0; iRead < NumReads; ++iRead)
        {
          // read X
          slaveSelect.setLevel(false);
          spi.write(reinterpret_cast<const std::byte*>(&ReadXCmd), 1);
          spi.transfer(reinterpret_cast<std::byte*>(rawSpiData), reinterpret_cast<const std::byte*>(zerosSpi), 2);
          slaveSelect.setLevel(true);
          xRaw[iRead] = (rawSpiData[0] << 4) | (rawSpiData[1] >> 3);
          // read Y
          slaveSelect.setLevel(false);
          spi.write(reinterpret_cast<const std::byte*>(&ReadYCmd), 1);
          spi.transfer(reinterpret_cast<std::byte*>(rawSpiData), reinterpret_cast<const std::byte*>(zerosSpi), 2);
          slaveSelect.setLevel(true);
          yRaw[iRead] = (rawSpiData[0] << 4) | (rawSpiData[1] >> 3);
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