#ifndef TFT_LCD_HPP
#define TFT_LCD_HPP

#include <cstdint>
#include <cstddef>

#include <STM32F4x1/Gpio.hpp>
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
  >
  class TftLcd
  {   
    public:
      using CanvasT =  EmbeddedGfx::UnbufferedCanvas<
                            Width, Height
                          , EmbeddedGfx::CanvasType::Normal
                          , EmbeddedGfx::ColorType::RGB565
                          , TftLcd>;
    public:
      TftLcd() : canvas_{*this}
      { }

      virtual void init() = 0;

      void reset()
      {
        using namespace std::chrono_literals;
        rstPin_.setLevel(true);
        Stm32::CycleCounter::delay(1ms);
        rstPin_.setLevel(false);
        Stm32::CycleCounter::delay(10ms);
        rstPin_.setLevel(true);
        Stm32::CycleCounter::delay(120ms);
      }

      void setCursor(const uint16_t x, const uint16_t y)
      {
        const uint8_t params1[] = {
                static_cast<uint8_t>(x >> 8)
              , static_cast<uint8_t>(x)
              , static_cast<uint8_t>(x >> 8)
              , static_cast<uint8_t>(x)
        };
        writeCmd(ColumnAddressSetCmd, params1, sizeof(params1));
        const uint8_t params2[] = {
                static_cast<uint8_t>(y >> 8)
              , static_cast<uint8_t>(y)
              , static_cast<uint8_t>(y >> 8)
              , static_cast<uint8_t>(y)
        };
        writeCmd(PageAddressSetCmd, params2, sizeof(params2));
      }

      void setWindow(const uint16_t xStart, const uint16_t yStart, const uint16_t xEnd, const uint16_t yEnd)
      {
        const uint8_t params1[] = {
                static_cast<uint8_t>(xStart >> 8)
              , static_cast<uint8_t>(xStart)
              , static_cast<uint8_t>(xEnd >> 8)
              , static_cast<uint8_t>(xEnd)
        };
        writeCmd(ColumnAddressSetCmd, params1, sizeof(params1));
        const uint8_t params2[] = {
                static_cast<uint8_t>(yStart >> 8)
              , static_cast<uint8_t>(yStart)
              , static_cast<uint8_t>(yEnd >> 8)
              , static_cast<uint8_t>(yEnd)
        };
        writeCmd(PageAddressSetCmd, params2, sizeof(params2));
      }

      void clear(const uint16_t color)
      {
        setWindow(0, 0, Width - 1, Height - 1);
        writeCmd(MemoryWriteCmd);
        for(size_t i = 0; i < Width * Height; ++i)
        {
          writeData(reinterpret_cast<const uint8_t*>(&color), 1);
        }
      }

      void setPixel(const size_t x, const size_t y, uint16_t value)
      {
        // set position for writing
        setCursor(x, y);
        // set data
        writeCmd(MemoryWriteCmd);
        writeData(reinterpret_cast<const uint8_t*>(&value), 1);
      }

      void setBacklight(const bool on)
      {
        blPin_.setLevel(on);
      }

      CanvasT& getCanvas() { return canvas_; }

    protected:
      void configurePins()
      {
        // pins mode: Output
        // speed for pins: High
        using namespace Stm32;
        Pins::setMode<
            CsPinT
          , DcPinT
          , RstPinT
          , BlPinT>(GpioMode::Output);

        Pins::setSpeed<
            CsPinT
          , DcPinT
          , RstPinT
          , BlPinT>(GpioSpeed::VeryHigh);

        Pins::setLevel<
            CsPinT
          , DcPinT
          , RstPinT
          , BlPinT>(0b0111);   
      }
      virtual void writeCmd(const uint8_t cmd, const uint8_t * const cmdParams = nullptr, const size_t numParams = 0) = 0;
      virtual void writeData(const uint8_t * const data, const size_t size) = 0;
    protected:
      CanvasT canvas_;
      CsPinT csPin_;
      DcPinT dcPin_;
      RstPinT rstPin_;
      BlPinT blPin_;
    protected:
      // commands
      static constexpr uint8_t SoftResetCmd                 = 0x01;
      static constexpr uint8_t ReadId                       = 0x04;
      static constexpr uint8_t ReadStatus                   = 0x09;
      static constexpr uint8_t SleepOutCmd                  = 0x11;
      static constexpr uint8_t DisplayOn                    = 0x29;
      static constexpr uint8_t ColumnAddressSetCmd          = 0x2A;
      static constexpr uint8_t PageAddressSetCmd            = 0x2B;
      static constexpr uint8_t MemoryWriteCmd               = 0x2C;
      static constexpr uint8_t MemoryAccessControlCmd       = 0x36;
      static constexpr uint8_t InterfacePixelFormatCmd      = 0x3A;
      static constexpr uint8_t InterfaceModeControlCmd      = 0xB0;
      static constexpr uint8_t FrameRateControlCmd          = 0xB1;
      static constexpr uint8_t DisplayInversionCmd          = 0xB4;
      static constexpr uint8_t DisplayFunctionCmd           = 0xB6;
      static constexpr uint8_t EntryModeSetCmd              = 0xB7;
      static constexpr uint8_t HsLanesControlCmd            = 0xBE;
      static constexpr uint8_t PowerControl1Cmd             = 0xC0;
      static constexpr uint8_t PowerControl2Cmd             = 0xC1;
      static constexpr uint8_t PowerControl3Cmd             = 0xC2;
      static constexpr uint8_t PowerControl4Cmd             = 0xC3;
      static constexpr uint8_t PowerControl5Cmd             = 0xC4;
      static constexpr uint8_t VcomControlCmd               = 0xC5;
      static constexpr uint8_t PositiveGammaControlCmd      = 0xE0;
      static constexpr uint8_t NegativeGammaCorrectionCmd   = 0xE1;
      static constexpr uint8_t DigitalGammaControl1Cmd      = 0xE2;
      static constexpr uint8_t DigitalGammaControl2Cmd      = 0xE3;
      static constexpr uint8_t SetImageFunctionCmd          = 0xE9;
  };
}

#endif //TFT_LCD_HPP