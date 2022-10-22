#include <cstdint>
#include <charconv>
#include <chrono>
#include <string_view>

#include <STM32F4x1/SerialUsb.hpp>
#include <STM32F4x1/CycleCounter.hpp>

class GuessNumber
{
  public:
    GuessNumber(Stm32::SerialUsb& serial)
      : running{true}
      , firstRun{true}
      , serial_{serial}
    {
      num = 12345;
    }

    void run()
    {
      using namespace std::literals;
      if(!running) return;
      if(firstRun)
      {
        Stm32::CycleCounter::delay(8s);
        printString("Welcome to Guess the Number!\r\n"sv);
        firstRun = false;
      }

      if(getInput())
      {
        if(input != num)
        {
          printString((input < num)
                      ? "Low, enter higher number.\r\n"sv
                      : "High, enter lower number.\r\n"sv);
        }
        else
        {
          printString("Congrats! Bye.\r\n"sv);
          running = false;
        }
      }
      else
      {
        printString("\r\nPlease give valid input.\r\n"sv);
      }      
    }

    void reset()
    {
      firstRun = true;
      running = true;
    }

    bool isRunning() const { return running; }
  private:
    void printString(const std::string_view str)
    {
      using namespace std::chrono_literals;
      serial_.write(
          reinterpret_cast<const std::byte*>(str.data())
        , str.size()
        , 1s
      );
    }

    bool getInput()
    {
      using namespace std::literals;
      printString("Enter number between 0 and 65535: "sv);

      static constexpr std::string_view expected{"\r\n"};
      char inputStr[10]{};
      size_t numRead = serial_.read(
          reinterpret_cast<std::byte*>(inputStr)
        , sizeof(inputStr)
        , reinterpret_cast<const std::byte*>(expected.data())
        , expected.size()
        , 10s
      );
      if(numRead < expected.size()) return false;
      const auto [ptr, ec] = std::from_chars(inputStr, inputStr + numRead - expected.size(), input);
      return (ec == std::errc());
    }
  private:
    uint16_t num, input;
    bool running, firstRun;
    Stm32::SerialUsb& serial_;
};

int main()
{
  using namespace Stm32;
  deviceInit();

  decltype(auto) serial = SerialUsb::getInstance();
  
  GuessNumber guessNumber{serial};

  while(true)
  {
    serial.update();
    if(serial.connected())
    {
      guessNumber.run();
    }
    else
    {
      guessNumber.reset();
    }
  }
}