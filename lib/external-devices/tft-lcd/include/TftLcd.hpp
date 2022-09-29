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
    , typename RdPinT
    , typename WrPinT
    , typename BlPinT
    , typename D0PinT
    , typename D1PinT
    , typename D2PinT
    , typename D3PinT
    , typename D4PinT
    , typename D5PinT
    , typename D6PinT
    , typename D7PinT
  >
  class TftLcd
  {
    private:
      CsPinT csPin_;
      DcPinT dcPin_;
      RstPinT rstPin_;
      RdPinT rdPin_;
      WrPinT wrPin_;
      BlPinT blPin_;
      D0PinT d0Pin_;
      D1PinT d1Pin_;
      D2PinT d2Pin_;
      D3PinT d3Pin_;
      D4PinT d4Pin_;
      D5PinT d5Pin_;
      D6PinT d6Pin_;
      D7PinT d7Pin_;
    public:
      using CanvasT =  EmbeddedGfx::UnbufferedCanvas<
                            Width, Height
                          , EmbeddedGfx::CanvasType::Normal
                          , EmbeddedGfx::ColorType::RGB565
                          , TftLcd>;
    public:
      TftLcd() : canvas_{*this}
      {
        // if anything needs to be constructed
      }

      void init()
      {
        using namespace std::chrono_literals;
        // configure pins
        configurePins();
        // reset
        reset();

        // init commands
        static constexpr uint8_t params1[] = {0x36, 0x04, 0x00, 0x3C, 0x0F, 0x8F};
        writeCmd(InitCmd1, params1, sizeof(params1));
        static constexpr uint8_t params2[] = {0x18, 0xA3, 0x12, 0x02, 0xB2, 0x12, 0xFF, 0x10, 0x00};
        writeCmd(InitCmd2, params2, sizeof(params2));
        static constexpr uint8_t params3[] = {0x21, 0x04};
        writeCmd(InitCmd3, params3, sizeof(params3));
        static constexpr uint8_t params4[] = {0x00, 0x08};
        writeCmd(InitCmd4, params4, sizeof(params4));
        static constexpr uint8_t params5[] = {0x08};
        writeCmd(MemoryAccessControlCmd, params5, sizeof(params5));
        static constexpr uint8_t params6[] = {0x00};
        writeCmd(DisplayInversionCmd, params6, sizeof(params6));
        static constexpr uint8_t params7[] = {0x47};
        writeCmd(PowerControl2Cmd, params7, sizeof(params7));
        static constexpr uint8_t params8[] = {0x00, 0xAF, 0x80, 0x00};
        writeCmd(VcomControlCmd, params8, sizeof(params8));
        static constexpr uint8_t params9[] = {0x0F, 0x1F, 0x1C, 0x0C, 0x0F, 0x08, 0x48, 0x98, 0x37, 0x0A, 0x13, 0x04, 0x11, 0x0D, 0x00};
        writeCmd(PositiveGammaControlCmd, params9, sizeof(params9)); 
        static constexpr uint8_t params10[] = {0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75, 0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00};
        writeCmd(NegativeGammaCorrectionCmd, params10, sizeof(params10));
        static constexpr uint8_t params11[] = {0x55};
        writeCmd(InterfacePixelFormatCmd, params11, sizeof(params11));
        writeCmd(SleepOutCmd, nullptr, 0);
        static constexpr uint8_t params12[] = {0x28};
        writeCmd(MemoryAccessControlCmd, params12, sizeof(params12));
        Stm32::CycleCounter::delay(120ms);
        static constexpr uint8_t params13[] = {(1<<6)|(1<<3)};
        writeCmd(MemoryAccessControlCmd, params13, sizeof(params13));
        writeCmd(DisplayOn, nullptr, 0);
        
        setBacklight(true);
        Stm32::CycleCounter::delay(4s);
        // for(size_t y = 0; y < Height; ++y)
        // {
        //   for(size_t x = 0; x < Width; ++x)
        //   {
        //     setPixel(x, y, 0xc0c0);
        //   }
        // }
        clear(0x5050);
      }

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
        writeCmd(MemoryWriteCmd, nullptr, 0);
        for(size_t i = 0; i < Width*Height; ++i)
        {
          writeData(reinterpret_cast<const uint8_t*>(&color), 1);
        }
      }

      void setPixel(const size_t x, const size_t y, uint16_t value)
      {
        // set position for writing
        setCursor(x, y);
        // set data
        writeCmd(MemoryWriteCmd, nullptr, 0);
        writeData(reinterpret_cast<const uint8_t*>(&value), 1);
      }

      void setBacklight(const bool on)
      {
        blPin_.setLevel(on);
      }

      CanvasT& getCanvas() { return canvas_; }

    private:
      void configurePins()
      {
        // pins mode: Output
        // speed for pins: High
        using namespace Stm32;
        Pins::setMode<
                    CsPinT
                  , DcPinT
                  , RstPinT
                  , RdPinT
                  , WrPinT
                  , BlPinT
                  , D0PinT
                  , D1PinT
                  , D2PinT
                  , D3PinT
                  , D4PinT
                  , D5PinT
                  , D6PinT
                  , D7PinT>(GpioMode::Output);

        Pins::setSpeed<
                    CsPinT
                  , DcPinT
                  , RstPinT
                  , RdPinT
                  , WrPinT
                  , BlPinT
                  , D0PinT
                  , D1PinT
                  , D2PinT
                  , D3PinT
                  , D4PinT
                  , D5PinT
                  , D6PinT
                  , D7PinT>(GpioSpeed::VeryHigh);

        Pins::setLevel<
                    CsPinT
                  , DcPinT
                  , RstPinT
                  , RdPinT
                  , WrPinT
                  , BlPinT
                  , D0PinT
                  , D1PinT
                  , D2PinT
                  , D3PinT
                  , D4PinT
                  , D5PinT
                  , D6PinT
                  , D7PinT>(0b00000000011111);          
      }
      void writeCmd(const uint8_t cmd, const uint8_t * const cmdParams, const size_t numParams)
      {
        using namespace std::chrono_literals;
        dcPin_.setLevel(false);
        csPin_.setLevel(false);
        Stm32::Pins::setLevel<
                    D0PinT
                  , D1PinT
                  , D2PinT
                  , D3PinT
                  , D4PinT
                  , D5PinT
                  , D6PinT
                  , D7PinT>(cmd);
        wrPin_.setLevel(false);
        wrPin_.setLevel(true);

        dcPin_.setLevel(true);
        for(uint8_t iParam = 0; iParam < numParams; ++iParam)
        {
          Stm32::Pins::setLevel<
                    D0PinT
                  , D1PinT
                  , D2PinT
                  , D3PinT
                  , D4PinT
                  , D5PinT
                  , D6PinT
                  , D7PinT>(cmdParams[iParam]);
          wrPin_.setLevel(false);
          wrPin_.setLevel(true);
        }
        csPin_.setLevel(true);
      }

      void writeData(const uint8_t * const data, const size_t size)
      {
        using namespace std::chrono_literals;
        dcPin_.setLevel(true);
        csPin_.setLevel(false);
        for(uint8_t iData = 0; iData < size; ++iData)
        {
          Stm32::Pins::setLevel<
                    D0PinT
                  , D1PinT
                  , D2PinT
                  , D3PinT
                  , D4PinT
                  , D5PinT
                  , D6PinT
                  , D7PinT>(data[iData]);
          wrPin_.setLevel(false);
          wrPin_.setLevel(true);
        }
        csPin_.setLevel(true);
      }
    private:
      CanvasT canvas_;
    private:
      // static constexpr commands
      static constexpr uint8_t SoftResetCmd = 0x01;
      static constexpr uint8_t InitCmd1 = 0xF1;
      static constexpr uint8_t InitCmd2 = 0xF2;
      static constexpr uint8_t InitCmd3 = 0xF8;
      static constexpr uint8_t InitCmd4 = 0xF9;
      static constexpr uint8_t MemoryAccessControlCmd = 0x36;
      static constexpr uint8_t DisplayInversionCmd = 0xB4;
      static constexpr uint8_t DisplayFunctionCmd = 0xB6;
      static constexpr uint8_t PowerControl1Cmd = 0xC0;
      static constexpr uint8_t PowerControl2Cmd = 0xC1;
      static constexpr uint8_t PowerControl3Cmd = 0xC2;
      static constexpr uint8_t PowerControl4Cmd = 0xC3;
      static constexpr uint8_t PowerControl5Cmd = 0xC4;
      static constexpr uint8_t VcomControlCmd = 0xC5;
      static constexpr uint8_t PositiveGammaControlCmd = 0xE0;
      static constexpr uint8_t NegativeGammaCorrectionCmd = 0xE1;
      static constexpr uint8_t DigitalGammaControl1Cmd = 0xE2;
      static constexpr uint8_t DigitalGammaControl2Cmd = 0xE3;
      static constexpr uint8_t SleepOutCmd = 0x11;
      static constexpr uint8_t DisplayOn = 0x29;
      static constexpr uint8_t ColumnAddressSetCmd = 0x2A;
      static constexpr uint8_t PageAddressSetCmd = 0x2B;
      static constexpr uint8_t MemoryWriteCmd = 0x2C;
      static constexpr uint8_t InterfaceModeControlCmd = 0xB0;
      static constexpr uint8_t InterfacePixelFormatCmd = 0x3A;
      static constexpr uint8_t NormalDisplayModeOnCmd = 0x13;
      static constexpr uint8_t DisplayInversionOffCmd = 0x20;
  };
}

#endif //TFT_LCD_HPP