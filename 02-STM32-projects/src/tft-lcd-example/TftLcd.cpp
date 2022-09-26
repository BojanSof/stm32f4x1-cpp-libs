#include <chrono>
#include <type_traits>

#include <Gpio.hpp>
#include <Clock.hpp>
#include <TftLcd.hpp>
#include <CycleCounter.hpp>

#include <EmbeddedGfx/Circle.hpp>
#include <EmbeddedGfx/Triangle.hpp>
#include <EmbeddedGfx/Text.hpp>

int main()
{
  using namespace Stm32;
  using namespace EmbeddedGfx;
  deviceInit();
  using namespace std::chrono_literals;
  CycleCounter::delay(200ms);
  using TftDisplay = Devices::TftLcd<
      320
    , 480
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
  >;  ///@todo add pins params
  TftDisplay lcd;
  lcd.init();
  lcd.setBacklight(true);
  auto& canvas = lcd.getCanvas();
  using CanvasT = std::remove_reference_t<decltype(canvas)>;
  Circle<CanvasT> circle{40, 40, 20};
  circle.setOutlineColor(Colors::White);
  canvas.draw(circle);
  Triangle<CanvasT> triangle{{{{10, 20}, {0, 40}, {110, 60}}}};
  triangle.setOutlineColor(Colors::White);
  canvas.draw(triangle);

  //canvas.clear();
  Text<100, Font<6, 8>, CanvasT> text{"LCD T E S T !!!", {10, 20}};
  text.setColor(Colors::White);
  canvas.draw(text);
  
  while(true)
  {
    
  }

  return 0;
}
