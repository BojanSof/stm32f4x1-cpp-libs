#ifndef STM32_I2C_RECEIVE_FSM_HPP
#define STM32_I2C_RECEIVE_FSM_HPP

#include <cstdint>
#include <stm32f4xx.h>

#include <FsmGenerator.hpp>

namespace Stm32
{
  template<uint8_t I2Cindex>
  class I2CReceiveFsm;  //< forward declaration

  template<uint8_t I2Cindex>
  class I2C;  //< forward declaration
}

template <uint8_t I2Cindex>
struct Fsm::States<Stm32::I2CReceiveFsm<I2Cindex>>
{
  enum class State
  {
    InvalidState,
    Idle,
    Start,
    SendAddr,
    ReceiveData,
    Nack,
    NackError,
    Stop
  };
};

namespace Stm32
{
  template<uint8_t I2Cindex>
  class I2CReceiveFsm : public Fsm::FsmGenerator<I2CReceiveFsm<I2Cindex>>
  {
    typedef typename Fsm::States<I2CReceiveFsm<I2Cindex>>::State State;
    
    public:
      I2CReceiveFsm() : Fsm::FsmGenerator<I2CReceiveFsm<I2Cindex>>(State::Idle)
      {
      }
    
    private:

      struct StartEvent : public Fsm::Event
      {
        size_t bytes;
      };
      
      struct AddrEvent : public Fsm::Event
      {
        uint8_t addr;
      };

      struct DataEvent : public Fsm::Event
      {
        std::byte* data;
      };

      struct StopEvent : public Fsm::Event
      {
      };

      struct LastDataEvent : public Fsm::Event
      {
        std::byte* data;
      };
      
      void doIdleToStart(const StartEvent& e)
      {
        if(e.bytes == 1)
        {
          i2cInstance_->SR1 &= ~I2C_CR1_ACK;
        }
        else
        {
          i2cInstance_->CR1 |= I2C_CR1_ACK;
        }
        i2cInstance_->CR1 |= I2C_CR1_START;
      }

      void doStartToSendAddr(const AddrEvent& e)
      {
        i2cInstance_->DR = e.addr;
      }

      void doReceiveData(const DataEvent& e)
      {
        *(e.data) = static_cast<std::byte>(i2cInstance_->DR);
      }
      
      void doSendNack(const LastDataEvent& e)
      {
        i2cInstance_->SR1 &= ~I2C_CR1_ACK;
        *(e.data) = static_cast<std::byte>(i2cInstance_->DR);
      }
      
      void doSendDataToStop(const StopEvent& e)
      {
        i2cInstance_->CR1 |= I2C_CR1_STOP;
      }

      friend class Fsm::FsmGenerator<I2CReceiveFsm<I2Cindex>>;
      friend class I2C<I2Cindex>;

      typedef typename Fsm::FsmGenerator<I2CReceiveFsm<I2Cindex>>::TransitionTable<
        typename Fsm::FsmGenerator<I2CReceiveFsm<I2Cindex>>::Transition<StartEvent, State::Idle, State::Start, &I2CReceiveFsm<I2Cindex>::doIdleToStart>
        , typename Fsm::FsmGenerator<I2CReceiveFsm<I2Cindex>>::Transition<AddrEvent, State::Start, State::SendAddr, &I2CReceiveFsm<I2Cindex>::doStartToSendAddr>
        , typename Fsm::FsmGenerator<I2CReceiveFsm<I2Cindex>>::Transition<DataEvent, State::SendAddr, State::ReceiveData, &I2CReceiveFsm<I2Cindex>::doReceiveData>
        , typename Fsm::FsmGenerator<I2CReceiveFsm<I2Cindex>>::Transition<LastDataEvent, State::ReceiveData, State::Nack, &I2CReceiveFsm<I2Cindex>::doSendNack>
        , typename Fsm::FsmGenerator<I2CReceiveFsm<I2Cindex>>::Transition<StopEvent, State::Nack, State::Stop, &I2CReceiveFsm<I2Cindex>::doSendDataToStop>
      > Table;
      I2C_TypeDef *const i2cInstance_ = I2C<I2Cindex>::getI2Cinstance();
};

}

#endif // STM32_I2C_RECEIVE_FSM_HPP