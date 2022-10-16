#include <STM32F4x1/SerialUsb.hpp>
#include <STM32F4x1/CycleCounter.hpp>

int main()
{
  using namespace Stm32;
  using namespace std::chrono_literals;
  deviceInit();

  decltype(auto) serial = SerialUsb::getInstance();
  
  const char msg[] = "Serial Test\r\n";
  const auto msgLen = sizeof(msg) - 1;

  while(true)
  {
    if(serial.connected())
    {
      serial.write(reinterpret_cast<const std::byte*>(msg), msgLen, 1s);
    }
    serial.update();
    CycleCounter::delay(500ms);
  }
}