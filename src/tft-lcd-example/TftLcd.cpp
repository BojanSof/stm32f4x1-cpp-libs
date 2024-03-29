#define SPI_DISPLAY 1  //< 1 to use SPI interface, 0 for GPIO interface, corresponding to display type
#define TOUCH_CALIBRATION 1 //< 1 to enable calibration, 0 to disable
#include <cstdio>
#include <chrono>
#include <type_traits>

#include <STM32F4x1/Gpio.hpp>
#include <STM32F4x1/Clock.hpp>
#include <STM32F4x1/SPI.hpp>
#include <STM32F4x1/CycleCounter.hpp>

#if SPI_DISPLAY == 1
#include <TftLcdSpi.hpp>
#else
#include <TftLcdGpio.hpp>
#endif
#include <Touch.hpp>

#if TOUCH_CALIBRATION == 1
#include <TouchCalibration.hpp>
#endif

#include <EmbeddedGfx/Circle.hpp>
#include <EmbeddedGfx/Rectangle.hpp>
#include <EmbeddedGfx/Triangle.hpp>
#include <EmbeddedGfx/Text.hpp>

int main()
{
  using namespace Stm32;
  using namespace EmbeddedGfx;
  deviceInit();
  using namespace std::chrono_literals;
  CycleCounter::delay(200ms);
  static constexpr size_t width = 320;
  static constexpr size_t height = 480;
#if SPI_DISPLAY == 1
  using TftDisplay = Devices::TftLcdSpi<
      width
    , height
    , Pins::PA15
    , Pins::PA9
    , Pins::PA12
    , Pins::PA8
    , Pins::PB5
    , Pins::PB4
    , Pins::PB3
    , SPI<1>
  >;
#else
  using TftDisplay = Devices::TftLcdGpio<
      width
    , height
    , Pins::PA8
    , Pins::PA9
    , Pins::PA12
    , Pins::PA11
    , Pins::PA10
    , Pins::PA15
    , Pins::PA0
    , Pins::PA1
    , Pins::PA2
    , Pins::PA3
    , Pins::PA4
    , Pins::PA5
    , Pins::PA6
    , Pins::PA7
  >;
#endif
  TftDisplay lcd;
  using MosiPin = Pins::PB15;
  using MisoPin = Pins::PB14;
  using SckPin = Pins::PB13;
  using SsPin = Pins::PB12;
  using Touch = Devices::Touch<
      width
    , height
    , SPI<2>
    , MosiPin
    , MisoPin
    , SckPin
    , SsPin
  >;
  Touch touch;
  lcd.init();
  lcd.setBacklight(true);
  auto& canvas = lcd.getCanvas();
  using CanvasT = std::remove_reference_t<decltype(canvas)>;

#if SPI_DISPLAY == 1
  touch.setOrientation(true, false);
#else
  touch.setOrientation(false, true);
#endif

  Circle<CanvasT> circle{200, 200, 30};
  circle.setOutlineColor(Colors::White);
  canvas.draw(circle);
  Triangle<CanvasT> triangle{{{{10, 20}, {0, 40}, {110, 60}}}};
  triangle.setOutlineColor(Colors::Red);
  canvas.draw(triangle);

  //canvas.clear();
  Text<100, Font<6, 8>, CanvasT> text{"LCD T E S T !!!", {10, 20}};
  text.setColor(Colors::White);
  canvas.draw(text);
#if TOUCH_CALIBRATION == 1
  // Calibrate touch screen
  using TouchCalibrationT = Devices::TouchCalibration<Touch, TftDisplay>;
  TouchCalibrationT calibration{touch, lcd};
  calibration.calibrate();
#endif
  
  while(true)
  {
    const auto touchData = touch.getCoordinates();
    if(touchData.has_value())
    {
      const auto& [x, y, z] = touchData.value();
      char buf[100]{};
      std::sprintf(buf, "X: %04d, Y: %04d, Z: %04d", x, y, z);
      Text<100, Font<6, 8>, CanvasT> text{buf, {10, 0}};
      text.setColor(Colors::White);
      // clear the upper portion of the screen
      Rectangle<CanvasT> rectangle{0, 0, width, 8};
      rectangle.setOutlineColor(Colors::Black);
      rectangle.setFillColor(Colors::Black);
      canvas.draw(rectangle);
      canvas.draw(text);
      canvas.setPixel(x, y, Colors::White);
    }

    // CycleCounter::delay(500ms);
  }

  return 0;
}
