#include <cstdint>
#include <chrono>
#include <ctime>

#include <STM32F4x1/SerialUsb.hpp>
#include <STM32F4x1/RealTimeClock.hpp>

int main()
{
  using namespace Stm32;
  deviceInit();

  decltype(auto) serial = SerialUsb::getInstance();
  
  // init RTC and set current time
  RealTimeClock::init();
  RealTimeClock::setTime(RealTimeClock::from_time_t(1667248530));
  std::time_t lastTimestamp = RealTimeClock::to_time_t(RealTimeClock::now());

  char str[100];
  while(true)
  {
    serial.update();
    const auto timestamp = RealTimeClock::to_time_t(RealTimeClock::now());
    if(serial.connected())
    {
      if(timestamp - lastTimestamp >= 2)
      {
        // auto [ptr, ec] = std::to_chars(str, str + sizeof(str), timestamp);
        // if(ec == std::errc())
        // {
        //   using namespace std::chrono_literals;
        //   str[ptr - str] = '\r';
        //   str[ptr - str + 1] = '\n';
        //   serial.write(reinterpret_cast<const std::byte*>(str), ptr - str + 2, 1s);
        // }
        size_t numChars{};
        if((numChars = std::strftime(str, sizeof(str), "%A %c", std::localtime(&timestamp))))
        {
          using namespace std::chrono_literals;
          str[numChars] = '\r';
          str[numChars + 1] = '\n';
          serial.write(reinterpret_cast<const std::byte*>(str), numChars + 2, 100ms);
        }
        lastTimestamp = RealTimeClock::to_time_t(RealTimeClock::now());
      }
    }
  }
}