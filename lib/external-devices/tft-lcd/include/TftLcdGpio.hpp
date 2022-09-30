#ifndef TFT_LCD_GPIO_HPP
#define TFT_LCD_GPIO_HPP

#include "TftLcd.hpp"

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
  class TftLcdGpio
    : public TftLcd<Width, Height, CsPinT, DcPinT, RstPinT, BlPinT>
  {
    using BaseT = TftLcd<Width, Height, CsPinT, DcPinT, RstPinT, BlPinT>;
    public:
      TftLcdGpio() { }

      void init() final
      {
        using namespace std::chrono_literals;
        // configure pins
        configurePins();
        // reset
        BaseT::reset();

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
        writeCmd(BaseT::MemoryAccessControlCmd, params5, sizeof(params5));
        static constexpr uint8_t params6[] = {0x00};
        writeCmd(BaseT::DisplayInversionCmd, params6, sizeof(params6));
        static constexpr uint8_t params7[] = {0x47};
        writeCmd(BaseT::PowerControl2Cmd, params7, sizeof(params7));
        static constexpr uint8_t params8[] = {0x00, 0xAF, 0x80, 0x00};
        writeCmd(BaseT::VcomControlCmd, params8, sizeof(params8));
        static constexpr uint8_t params9[] = {0x0F, 0x1F, 0x1C, 0x0C, 0x0F, 0x08, 0x48, 0x98, 0x37, 0x0A, 0x13, 0x04, 0x11, 0x0D, 0x00};
        writeCmd(BaseT::PositiveGammaControlCmd, params9, sizeof(params9)); 
        static constexpr uint8_t params10[] = {0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75, 0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00};
        writeCmd(BaseT::NegativeGammaCorrectionCmd, params10, sizeof(params10));
        static constexpr uint8_t params11[] = {0x55};
        writeCmd(BaseT::InterfacePixelFormatCmd, params11, sizeof(params11));
        writeCmd(BaseT::SleepOutCmd);
        static constexpr uint8_t params12[] = {0x28};
        writeCmd(BaseT::MemoryAccessControlCmd, params12, sizeof(params12));
        Stm32::CycleCounter::delay(120ms);
        static constexpr uint8_t params13[] = {(1<<6)|(1<<3)};
        writeCmd(BaseT::MemoryAccessControlCmd, params13, sizeof(params13));
        writeCmd(BaseT::DisplayOn);

        BaseT::setBacklight(true);
        Stm32::CycleCounter::delay(4s);
        BaseT::clear(0x5050);
      }

    private:
      void configurePins()
      {
        BaseT::configurePins();
        using namespace Stm32;
        Pins::setMode<
            RdPinT
          , WrPinT
          , D0PinT
          , D1PinT
          , D2PinT
          , D3PinT
          , D4PinT
          , D5PinT
          , D6PinT
          , D7PinT>(GpioMode::Output);

        Pins::setSpeed<
            RdPinT
          , WrPinT
          , D0PinT
          , D1PinT
          , D2PinT
          , D3PinT
          , D4PinT
          , D5PinT
          , D6PinT
          , D7PinT>(GpioSpeed::VeryHigh);

        Pins::setLevel<
            RdPinT
          , WrPinT
          , D0PinT
          , D1PinT
          , D2PinT
          , D3PinT
          , D4PinT
          , D5PinT
          , D6PinT
          , D7PinT>(0b0000000011);          
      }

      void writeCmd(const uint8_t cmd, const uint8_t * const cmdParams = nullptr, const size_t numParams = 0) final
      {
        using namespace std::chrono_literals;
        BaseT::dcPin_.setLevel(false);
        BaseT::csPin_.setLevel(false);
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

        BaseT::dcPin_.setLevel(true);
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
        BaseT::csPin_.setLevel(true);
      }

      void writeData(const uint8_t * const data, const size_t size) final
      {
        using namespace std::chrono_literals;
        BaseT::dcPin_.setLevel(true);
        BaseT::csPin_.setLevel(false);
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
        BaseT::csPin_.setLevel(true);
      }
    private:
      RdPinT rdPin_;
      WrPinT wrPin_;
      D0PinT d0Pin_;
      D1PinT d1Pin_;
      D2PinT d2Pin_;
      D3PinT d3Pin_;
      D4PinT d4Pin_;
      D5PinT d5Pin_;
      D6PinT d6Pin_;
      D7PinT d7Pin_;
    private:
      // commands
      static constexpr uint8_t InitCmd1 = 0xF1;
      static constexpr uint8_t InitCmd2 = 0xF2;
      static constexpr uint8_t InitCmd3 = 0xF8;
      static constexpr uint8_t InitCmd4 = 0xF9;
  };
}

#endif //TFT_LCD_GPIO_HPP