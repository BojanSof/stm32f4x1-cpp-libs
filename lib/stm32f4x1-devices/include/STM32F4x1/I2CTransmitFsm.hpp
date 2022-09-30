#ifndef STM32_I2C_TRANSMIT_FSM_HPP
#define STM32_I2C_TRANSMIT_FSM_HPP

#include <cstdint>

#include <stm32f4xx.h>

#include <FsmGenerator/FsmGenerator.hpp>

namespace Stm32
{
  template<uint8_t I2Cindex>
  class I2CTransmitFsm;  //< forward declaration

  template<uint8_t I2Cindex>
  class I2C;  //< forward declaration
}

template <uint8_t I2Cindex>
struct Fsm::States<Stm32::I2CTransmitFsm<I2Cindex>>
{
  enum class State
  {
    InvalidState,
    Idle,
    Start,
    SendAddr,
    SendData,
    NackError,
    Stop
  };
};

namespace Stm32
{
  template<uint8_t I2Cindex>
  class I2CTransmitFsm : public Fsm::FsmGenerator<I2CTransmitFsm<I2Cindex>>
  {
    typedef typename Fsm::States<I2CTransmitFsm<I2Cindex>>::State State;
    
    public:
      I2CTransmitFsm() : Fsm::FsmGenerator<I2CTransmitFsm<I2Cindex>>(State::Idle)
      {
      }
    
    private:

      struct StartEvent : public Fsm::Event
      {
      };
      
      struct AddrEvent : public Fsm::Event
      {
        uint8_t addr;
      };

      struct DataEvent : public Fsm::Event
      {
        uint8_t data;
      };

      struct StopEvent : public Fsm::Event
      {
      };
      
      void doIdleToStart(const StartEvent& e)
      {
        i2cInstance_->CR1 |= I2C_CR1_START;
      }

      void doStartToSendAddr(const AddrEvent& e)
      {
        i2cInstance_->DR = e.addr;
      }

      void doSendData(const DataEvent& e)
      {
        i2cInstance_->DR = e.data;
      }
      
      void doSendDataToStop(const StopEvent& e)
      {
        i2cInstance_->CR1 |= I2C_CR1_STOP;
      }

      friend class Fsm::FsmGenerator<I2CTransmitFsm<I2Cindex>>;
      friend class I2C<I2Cindex>;

      typedef typename Fsm::FsmGenerator<I2CTransmitFsm<I2Cindex>>::TransitionTable<
        typename Fsm::FsmGenerator<I2CTransmitFsm<I2Cindex>>::Transition<StartEvent, State::Idle, State::Start, &I2CTransmitFsm<I2Cindex>::doIdleToStart>
        , typename Fsm::FsmGenerator<I2CTransmitFsm<I2Cindex>>::Transition<AddrEvent, State::Start, State::SendAddr, &I2CTransmitFsm<I2Cindex>::doStartToSendAddr>
        , typename Fsm::FsmGenerator<I2CTransmitFsm<I2Cindex>>::Transition<DataEvent, State::SendAddr, State::SendData, &I2CTransmitFsm<I2Cindex>::doSendData>
        , typename Fsm::FsmGenerator<I2CTransmitFsm<I2Cindex>>::Transition<DataEvent, State::SendData, State::SendData, &I2CTransmitFsm<I2Cindex>::doSendData>
        , typename Fsm::FsmGenerator<I2CTransmitFsm<I2Cindex>>::Transition<StopEvent, State::SendData, State::Stop, &I2CTransmitFsm<I2Cindex>::doSendDataToStop>
      > Table;
      I2C_TypeDef *const i2cInstance_ = I2C<I2Cindex>::getI2Cinstance();
};

}

#endif // STM32_I2C_TRANSMIT_FSM_HPP