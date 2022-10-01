#ifndef TFT_LCD_HPP
#define TFT_LCD_HPP

#include <cstdint>
#include <cstddef>

#include "TftLcdCommands.hpp"

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
    , typename TftLcdDerivedT
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

      void init()
      {
        static_cast<TftLcdDerivedT&>(*this).init();
      }

      void setPixel(const size_t x, const size_t y, uint16_t value)
      {
        static_cast<TftLcdDerivedT&>(*this).setPixel(x, y, value);
      }

      void clear(uint16_t value)
      {
        static_cast<TftLcdDerivedT&>(*this).clear(value);
      }

      CanvasT& getCanvas() { return canvas_; }
      
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

      void setBacklight(const bool on)
      {
        blPin_.setLevel(on);
      }

      void setCursor(const uint16_t x, const uint16_t y)
      {
        const uint8_t params1[] = {
                static_cast<uint8_t>(x >> 8)
              , static_cast<uint8_t>(x)
              , static_cast<uint8_t>(x >> 8)
              , static_cast<uint8_t>(x)
        };
        writeCmd(TftLcdCommands::ColumnAddressSet, params1, sizeof(params1));
        const uint8_t params2[] = {
                static_cast<uint8_t>(y >> 8)
              , static_cast<uint8_t>(y)
              , static_cast<uint8_t>(y >> 8)
              , static_cast<uint8_t>(y)
        };
        writeCmd(TftLcdCommands::PageAddressSet, params2, sizeof(params2));
      }

      void setWindow(const uint16_t xStart, const uint16_t yStart, const uint16_t xEnd, const uint16_t yEnd)
      {
        const uint8_t params1[] = {
                static_cast<uint8_t>(xStart >> 8)
              , static_cast<uint8_t>(xStart)
              , static_cast<uint8_t>(xEnd >> 8)
              , static_cast<uint8_t>(xEnd)
        };
        writeCmd(TftLcdCommands::ColumnAddressSet, params1, sizeof(params1));
        const uint8_t params2[] = {
                static_cast<uint8_t>(yStart >> 8)
              , static_cast<uint8_t>(yStart)
              , static_cast<uint8_t>(yEnd >> 8)
              , static_cast<uint8_t>(yEnd)
        };
        writeCmd(TftLcdCommands::PageAddressSet, params2, sizeof(params2));
      }

      void setOrientation(const bool my, const bool mx, const bool mv, const bool bgr)
      {
        using namespace TftLcdCommands;
        uint8_t param = (bgr ? MemoryAccessControlBGR : 0)
                      | (my ? MemoryAccessControlMY : 0)
                      | (mx ? MemoryAccessControlMX : 0)
                      | (mv ? MemoryAccessControlMV : 0);
        writeCmd(MemoryAccessControl, &param, sizeof(param));
      }

    protected:
      void writeCmd(const uint8_t cmd, const uint8_t * const cmdParams = nullptr, const size_t numParams = 0)
      {
        static_cast<TftLcdDerivedT&>(*this).writeCmd(cmd, cmdParams, numParams);
      }
      void writeData(const uint8_t * const data, const size_t size)
      {
        static_cast<TftLcdDerivedT&>(*this).writeData(data, size);
      }

      void configurePins()
      {
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
    protected:
      CanvasT canvas_;
      CsPinT csPin_;
      DcPinT dcPin_;
      RstPinT rstPin_;
      BlPinT blPin_;
  };
}

#endif //TFT_LCD_HPP