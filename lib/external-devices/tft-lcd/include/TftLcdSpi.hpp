#ifndef TFT_LCD_SPI_HPP
#define TFT_LCD_SPI_HPP

#include "TftLcd.hpp"

#include <STM32F4x1/SPI.hpp>
#include <STM32F4x1/CycleCounter.hpp>

namespace Devices
{
  template<
      size_t Width
    , size_t Height
    , typename CsPinT
    , typename DcPinT
    , typename RstPinT
    , typename BlPinT
    , typename MosiPinT
    , typename MisoPinT
    , typename SckPinT
    , typename SpiInterfaceT
  >
  class TftLcdSpi
    : public TftLcd<Width, Height, CsPinT, DcPinT, RstPinT, BlPinT>
  {
    using BaseT = TftLcd<Width, Height, CsPinT, DcPinT, RstPinT, BlPinT>;
    public:
      TftLcdSpi() { }

      void init() final
      {
        using namespace std::chrono_literals;
        using namespace Stm32;
        // configure pins
        BaseT::configurePins();
        // configure SPI
        auto& spi = SpiInterfaceT::getInstance();
        using ConfigPins = SpiPins<MosiPinT, MisoPinT, SckPinT, CsPinT>;
        using Config = SpiConfig<
            10000000ul       //< SCK freq
          , SpiMode::Mode0   //< SPI mode
          , false            //< 8-bit frame size
          , false            //< send MSB first
          , ConfigPins       //< SPI pins
          , true             //< software SS control
        >;
        spi.template configure<Config>();
        // reset
        BaseT::reset();

        // init commands
        static constexpr uint8_t params1[] = {0xA9, 0x51, 0x2C, 0x82};
        writeCmd(InitCmd1, params1, sizeof(params1));
        static constexpr uint8_t params2[] = {0x11, 0x09};
        writeCmd(BaseT::PowerControl1Cmd, params2, sizeof(params2));
        static constexpr uint8_t params3[] = {0x41};
        writeCmd(BaseT::PowerControl2Cmd, params3, sizeof(params3));
        static constexpr uint8_t params4[] = {0x00, 0x0A, 0x80};
        writeCmd(BaseT::VcomControlCmd, params4, sizeof(params4));
        static constexpr uint8_t params5[] = {0xB0, 0x11};
        writeCmd(BaseT::FrameRateControlCmd, params5, sizeof(params5));
        static constexpr uint8_t params6[] = {0x02};
        writeCmd(BaseT::DisplayInversionCmd, params6, sizeof(params6));
        static constexpr uint8_t params7[] = {0x02, 0x42};
        writeCmd(BaseT::DisplayFunctionCmd, params7, sizeof(params7));
        static constexpr uint8_t params8[] = {0xC6};
        writeCmd(BaseT::EntryModeSetCmd, params8, sizeof(params8));
        static constexpr uint8_t params9[] = {0x00, 0x04};
        writeCmd(BaseT::HsLanesControlCmd, params9, sizeof(params9));
        static constexpr uint8_t params10[] = {0x00};
        writeCmd(BaseT::SetImageFunctionCmd, params10, sizeof(params10));
        static constexpr uint8_t params11[] = {0x55};
        writeCmd(BaseT::InterfacePixelFormatCmd, params11, sizeof(params11));
        static constexpr uint8_t params12[] = {0x00, 0x07, 0x10, 0x09, 0x17, 0x0B, 0x41, 0x89, 0x4B, 0x0A, 0x0C, 0x0E, 0x18, 0x1B, 0x0F};
        writeCmd(BaseT::PositiveGammaControlCmd, params12, sizeof(params12)); 
        static constexpr uint8_t params13[] = {0x00, 0x17, 0x1A, 0x04, 0x0E, 0x06, 0x2F, 0x45, 0x43, 0x02, 0x0A, 0x09, 0x32, 0x36, 0x0F};
        writeCmd(BaseT::NegativeGammaCorrectionCmd, params13, sizeof(params13));
        writeCmd(BaseT::SleepOutCmd);
        Stm32::CycleCounter::delay(120ms);
        static constexpr uint8_t params14[] = {(1<<6)|(1<<3)};
        writeCmd(BaseT::MemoryAccessControlCmd, params14, sizeof(params14));
        writeCmd(BaseT::DisplayOn);

        BaseT::setBacklight(true);
        // Stm32::CycleCounter::delay(4s);
        BaseT::clear(0x5050);
      }
    private:
      void writeCmd(const uint8_t cmd, const uint8_t * const cmdParams = nullptr, const size_t numParams = 0) final
      {
        auto& spi = SpiInterfaceT::getInstance();
        BaseT::dcPin_.setLevel(false);
        spi.write(reinterpret_cast<const std::byte*>(&cmd), sizeof(uint8_t));

        BaseT::dcPin_.setLevel(true);
        for(uint8_t iParam = 0; iParam < numParams; ++iParam)
        {
          spi.write(reinterpret_cast<const std::byte*>(&cmdParams[iParam]), sizeof(uint8_t));
        }
      }

      void writeData(const uint8_t * const data, const size_t size) final
      {
        auto& spi = SpiInterfaceT::getInstance();
        BaseT::dcPin_.setLevel(true);
        for(uint8_t iData = 0; iData < size; ++iData)
        {
          spi.write(reinterpret_cast<const std::byte*>(&data[iData]), sizeof(uint8_t));
        }
      }
    private:
      // commands
      static constexpr uint8_t InitCmd1 = 0xF7;
  };
}

#endif //TFT_LCD_SPI_HPP