#include <chrono>
#include <type_traits>

#include <STM32F4x1/Gpio.hpp>
#include <STM32F4x1/Clock.hpp>
#include <STM32F4x1/I2C.hpp>
#include <STM32F4x1/CycleCounter.hpp>
#include <OledDisplay.hpp>

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
  using  OledDisplay = Devices::OledDisplay<I2C<1>, 0x3C, Pins::PB7, Pins::PB6>;
  OledDisplay oled;
  oled.init();
  auto& canvas = oled.getCanvas();
  using CanvasT = std::remove_reference_t<decltype(canvas)>;
  Circle<CanvasT> circle{40, 40, 20};
  circle.setOutlineColor(Colors::White);
  canvas.draw(circle);
  Triangle<CanvasT> triangle{{{{10, 20}, {0, 40}, {110, 60}}}};
  triangle.setOutlineColor(Colors::White);
  canvas.draw(triangle);
  oled.updateScreen();

  //canvas.clear();
  Text<100, Font<6, 8>, CanvasT> text{"OLED T E S T !!!", {10, 20}};
  text.setColor(Colors::White);
  canvas.draw(text);
  oled.updateScreen();
  
  while(true)
  {
    
  }

  return 0;
}
