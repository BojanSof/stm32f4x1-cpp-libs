#ifndef TFT_LCD_HPP
#define TFT_LCD_HPP

#include <cstdint>
#include <cstddef>
#include <Gpio.hpp>
#include <CycleCounter.hpp>

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
      TftLcd()
      {
        // if anything needs to be constructed
      }

      void init()
      {
        // configure pins
        configurePins();
        // reset
        reset();
        // init commands
      }

      void reset()
      {
        rstPin_.setLevel(true);
        delay(1ms);
        rstPin_.setLevel(false);
        delay(10ms);
        rstPin_.setLevel(true);
        delay(120ms);
      }

      void setPixel(const size_t x, const size_t y, uint16_t value)
      {
        // set position for writing
        // set data
      }

      void setBacklight(const bool on)
      {
        // turn backlight on or off (BlPin)
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
                  , D7PinT>(GpioSpeed::High);

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
                  , D7PinT>(0b10111000000000);          
      }

      void writeCmd(const uint8_t cmd, const uint8_t * const cmdParams, const size_t numParams)
      {
        csPin_.setLevel(false);
        dcPin_.setLevel(false);
        wrPin_.setLevel(false);
        Pins::setLevel<
                    D0PinT
                  , D1PinT
                  , D2PinT
                  , D3PinT
                  , D4PinT
                  , D5PinT
                  , D6PinT
                  , D7PinT>(cmd);
        wrPin_.setLevel(true);
        dcPin_.setLevel(true);
        for(uint8_t iParam = 0; iParam < numParams; ++iParam)
        {
          wrPin_.setLevel(false);
          Pins::setLevel<
                    D0PinT
                  , D1PinT
                  , D2PinT
                  , D3PinT
                  , D4PinT
                  , D5PinT
                  , D6PinT
                  , D7PinT>(cmdParams[iParam]);
          wrPin_.setLevel(true);
        }
        csPin_.setLevel(true);
        dcPin_.setLevel(false);
      }

      void writeData(const uint8_t * const data, const size_t size)
      {

      }
    private:
      CanvasT canvas_;
    private:
      // static constexpr commands
      
  };
}

#endif //TFT_LCD_HPP