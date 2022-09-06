#include <Gpio.hpp>
#include <Clock.hpp>
#include <I2C.hpp>
#include <OledDisplay.hpp>
#include <Circle.hpp>
#include <Triangle.hpp>
#include <CycleCounter.hpp>
#include <chrono>
#include <type_traits>
#include <Text.hpp>

int main()
{
  using namespace Stm32;
  using namespace EmbeddedGfx;
  deviceInit();
  using namespace std::chrono_literals;
  CycleCounter::delay(200ms);
  using  OledDisplay = Devices::OledDisplay<I2C<1>, 0x3C, Pins::PB7, Pins::PB6>;
  OledDisplay oled;
  oled.init();
  auto& canvas = oled.getCanvas();
  using CanvasT = std::remove_reference_t<decltype(canvas)>;
  Circle<CanvasT> circle(40, 40, 20);
  canvas.draw(circle);
  Triangle<CanvasT> triangle({{{10, 20}, {0, 40}, {110, 60}}});
  canvas.draw(triangle);
  oled.updateScreen();

  //canvas.clear();
  Text<100, Font<6, 8>, CanvasT> text1("S W C E N C E   M O E");
  Text<100, Font<6, 8>, CanvasT> text2("OLED T E S T !!!", {10, 20});
  canvas.draw(text1);
  canvas.draw(text2);
  oled.updateScreen();
  
  while(true)
  {
    
  }

  return 0;
}
