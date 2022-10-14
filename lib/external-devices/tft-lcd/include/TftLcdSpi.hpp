#ifndef TFT_LCD_SPI_HPP
#define TFT_LCD_SPI_HPP

#include <cassert>

#include "TftLcd.hpp"

#include <STM32F4x1/SPI.hpp>
#include <STM32F4x1/CycleCounter.hpp>
#include <EmbeddedGfx/UnbufferedCanvas.hpp>

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
    : public TftLcd<Width, Height, CsPinT, DcPinT, RstPinT, BlPinT
                  , TftLcdSpi<Width, Height, CsPinT, DcPinT, RstPinT, BlPinT, MosiPinT, MisoPinT, SckPinT, SpiInterfaceT> >
  {
    using BaseT = TftLcd<Width, Height, CsPinT, DcPinT, RstPinT, BlPinT, TftLcdSpi>;
    friend BaseT;
    using CanvasT = EmbeddedGfx::UnbufferedCanvas<
                            Width, Height
                          , EmbeddedGfx::CanvasType::Normal
                          , EmbeddedGfx::RGB666
                          , TftLcdSpi>;
    public:
      TftLcdSpi() : canvas_{*this}
      { }

      void init()
      {
        using namespace std::chrono_literals;
        using namespace Stm32;
        // configure pins
        BaseT::configurePins();
        // configure SPI
        auto& spi = SpiInterfaceT::getInstance();
        using ConfigPins = SpiPins<MosiPinT, MisoPinT, SckPinT, CsPinT>;
        using Config = SpiConfig<
#ifdef NDEBUG
            20000000ul       //< SCK freq        
#else
            2000000ul        //< SCK freq
#endif
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
        writeCmd(TftLcdCommands::AdjustControl3, params1, sizeof(params1));
        static constexpr uint8_t params2[] = {0x11, 0x09};
        writeCmd(TftLcdCommands::PowerControl1, params2, sizeof(params2));
        static constexpr uint8_t params3[] = {0x41};
        writeCmd(TftLcdCommands::PowerControl2, params3, sizeof(params3));
        static constexpr uint8_t params4[] = {0x00, 0x0A, 0x80};
        writeCmd(TftLcdCommands::VcomControl, params4, sizeof(params4));
        static constexpr uint8_t params5[] = {0xA0};
        writeCmd(TftLcdCommands::FrameRateControlNormalMode, params5, sizeof(params5));
        static constexpr uint8_t params6[] = {0x02};
        writeCmd(TftLcdCommands::DisplayInversion, params6, sizeof(params6));
        static constexpr uint8_t params7[] = {0x02, 0x22};
        writeCmd(TftLcdCommands::DisplayFunctionControl, params7, sizeof(params7));
        static constexpr uint8_t params8[] = {0xC6};
        writeCmd(TftLcdCommands::EntryModeSet, params8, sizeof(params8));
        static constexpr uint8_t params9[] = {0x00, 0x04};
        writeCmd(TftLcdCommands::HsLanesControl, params9, sizeof(params9));
        static constexpr uint8_t params10[] = {0x00};
        writeCmd(TftLcdCommands::SetImageFunction, params10, sizeof(params10));
        static constexpr uint8_t params11[] = {0x66};
        writeCmd(TftLcdCommands::InterfacePixelFormat, params11, sizeof(params11));
        static constexpr uint8_t params12[] = {0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F};
        writeCmd(TftLcdCommands::PositiveGammaControl, params12, sizeof(params12));
        static constexpr uint8_t params13[] = {0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F};
        writeCmd(TftLcdCommands::NegativeGammaCorrection, params13, sizeof(params13));
        writeCmd(TftLcdCommands::SleepOut);
        Stm32::CycleCounter::delay(120ms);
        setOrientation(true);
        writeCmd(TftLcdCommands::DisplayOn);

        BaseT::setBacklight(true);
        clear(0);
      }

      void clear(const uint32_t color)
      {
        BaseT::setWindow(0, 0, Width - 1, Height - 1);
        writeCmd(TftLcdCommands::MemoryWrite);
        for(size_t i = 0; i < Width * Height; ++i)
        {
          const uint8_t colorBytes[] = {
              static_cast<uint8_t>(color >> 16)
            , static_cast<uint8_t>(color >> 8)
            , static_cast<uint8_t>(color)
          };
          writeData(colorBytes, sizeof(colorBytes));
        }
      }

      void setPixel(const size_t x, const size_t y, uint32_t value)
      {
        // set position for writing
        BaseT::setCursor(x, y);
        // set data
        writeCmd(TftLcdCommands::MemoryWrite);
        const uint8_t colorBytes[] = {
              static_cast<uint8_t>(value >> 16)
            , static_cast<uint8_t>(value >> 8)
            , static_cast<uint8_t>(value)
          };
        writeData(colorBytes, sizeof(colorBytes));
      }

      void setOrientation(const bool flip = false)
      {
        if constexpr (Width > Height)
        {
          // landscape mode
          BaseT::setOrientation(flip, !flip, true, true);
        }
        else
        {
          // portrait mode
          BaseT::setOrientation(flip, flip, false, true);
        }
      }

      CanvasT& getCanvas() { return canvas_; }
      
    private:
      void writeCmd(const uint8_t cmd, const uint8_t * const cmdParams = nullptr, const size_t numParams = 0)
      {
        auto& spi = SpiInterfaceT::getInstance();
        BaseT::dcPin_.setLevel(false);
        spi.transfer(nullptr, reinterpret_cast<const std::byte*>(&cmd), sizeof(uint8_t));

        BaseT::dcPin_.setLevel(true);
        for(uint8_t iParam = 0; iParam < numParams; ++iParam)
        {
          spi.transfer(nullptr, reinterpret_cast<const std::byte*>(&cmdParams[iParam]), sizeof(uint8_t));
        }
      }

      void writeData(const uint8_t * const data, const size_t size)
      {
        auto& spi = SpiInterfaceT::getInstance();
        BaseT::dcPin_.setLevel(true);
        for(uint8_t iData = 0; iData < size; ++iData)
        {
          spi.transfer(nullptr, reinterpret_cast<const std::byte*>(&data[iData]), sizeof(uint8_t));
        }
      }
    private:
      CanvasT canvas_;
  };
}

#endif //TFT_LCD_SPI_HPP