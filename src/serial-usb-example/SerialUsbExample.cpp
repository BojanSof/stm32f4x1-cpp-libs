#include <cstdint>
#include <chrono>
#include <string_view>

#include <STM32F4x1/SerialUsb.hpp>
#include <STM32F4x1/CycleCounter.hpp>

#include <FsmGenerator/FsmGenerator.hpp>

using namespace Fsm;

class GuessNumberFsm;

template<>
struct Fsm::States<GuessNumberFsm>
{
  enum class State
  {
    Greet,
    InvalidState,
    WaitForInput,
    CheckInput,
    End
  };
};

class GuessNumberFsm
  : public FsmGenerator<GuessNumberFsm>
{
  public:
    using State = typename States<GuessNumberFsm>::State;

    GuessNumberFsm(Stm32::SerialUsb& serial)
      : FsmGenerator{State::CheckInput}
      , running{true}
      , serial_{serial}
    {
      num = 12345;
      processEvent(GreetEvent{});
    }

    void run()
    {
      if(running)
      {
        processEvent(RetryEvent{});
        InputEvent in;
        in.number = input;
        processEvent(in);
        if(!running) processEvent(EndEvent{});
      }
    }
  private:
    uint16_t num, input;
    bool running;
    Stm32::SerialUsb& serial_;
  private:
    friend class FsmGenerator<GuessNumberFsm>;

    struct GreetEvent : public Event {};
    struct InputEvent : public Event
    {
      unsigned int number;
    };
    struct RetryEvent : public Event {};
    struct EndEvent : public Event {};

    void doGreetCheckInput(const GreetEvent& e)
    {
      (void)e;
      static constexpr std::string_view msg{"Welcome to Guess the number."};
      using namespace std::chrono_literals;
      serial_.write(
          reinterpret_cast<const std::byte*>(msg.data())
        , msg.size()
        , 1s
      );
    }

    void doWaitForInputCheckInput(const InputEvent& e)
    {
      if(e.number != num)
      {
        using namespace std::chrono_literals;
        const std::string_view msg{(e.number < num) ? "Low, enter higher number." : "High, enter lower number."};
        serial_.write(
            reinterpret_cast<const std::byte*>(msg.data())
          , msg.size()
          , 1s
        );
      }
      else
      {
        running = false;
      }
    }

    void doCheckInputWaitForInput(const RetryEvent& e)
    {
      (void)e;
      using namespace std::chrono_literals;
      std::string_view msg{"Enter number between 0 and 65535: "};
      serial_.write(
          reinterpret_cast<const std::byte*>(msg.data())
        , msg.size()
        , 1s
      );
      static constexpr std::string_view expected{"\n"};
      serial_.read(
          reinterpret_cast<std::byte*>(&input)
        , sizeof(input)
        , reinterpret_cast<const std::byte*>(expected.data())
        , expected.size()
        , 10s
      );
    }

    void doCheckInputStop(const EndEvent& e)
    {
      (void)e;
      using namespace std::chrono_literals;
      static constexpr std::string_view msg{"Congrats! Bye."};
      serial_.write(
          reinterpret_cast<const std::byte*>(msg.data())
        , msg.size()
        , 1s
      );
    }

    using Table = TransitionTable<
        Transition<GreetEvent, State::Greet, State::CheckInput, &GuessNumberFsm::doGreetCheckInput>
      , Transition<InputEvent, State::WaitForInput, State::CheckInput, &GuessNumberFsm::doWaitForInputCheckInput>
      , Transition<RetryEvent, State::CheckInput, State::WaitForInput, &GuessNumberFsm::doCheckInputWaitForInput>
      , Transition<EndEvent, State::CheckInput, State::End, &GuessNumberFsm::doCheckInputStop>
    >;
};

int main()
{
  using namespace Stm32;
  deviceInit();

  decltype(auto) serial = SerialUsb::getInstance();
  
  GuessNumberFsm guessNumber{serial};

  while(true)
  {
    if(serial.connected())
    {
      guessNumber.run();
    }
    serial.update();
  }
}