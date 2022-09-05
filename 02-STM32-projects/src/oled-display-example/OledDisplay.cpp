#include <Gpio.hpp>
#include <Clock.hpp>
#include <I2C.hpp>
#include <OledDisplay.hpp>
#include <Circle.hpp>
#include <CycleCounter.hpp>
#include <chrono>
#include <type_traits>

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
  Circle<std::remove_reference_t<decltype(canvas)>> circle(40, 40, 20);
  canvas.draw(circle);
  oled.updateScreen();
  
  while(true)
  {
    
  }

  return 0;
}
