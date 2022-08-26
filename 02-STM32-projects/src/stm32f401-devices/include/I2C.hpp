#ifndef STM32_I2C_HPP
#define STM32_I2C_HPP

#include <cstdint>
#include <stm32f4xx.h>

#include "ComInterface.hpp"
#include "Clock.hpp"
#include "Gpio.hpp"

namespace Stm32
{
  extern bool ensureI2CLink; //< ensure that the source file with the IRQ handlers is linked
  template <
    uint32_t ClockFrequency
    , uint8_t SlaveAddress
    , typename SdaPin
    , typename SclPin>
  struct I2Cconfig : public ComConfig
  {
    static constexpr uint32_t clockFrequency = ClockFrequency;
    static constexpr uint8_t slaveAddress = SlaveAddress;
    using sdaPin = SdaPin;
    using sclPin = SclPin;
  };

  template < uint8_t I2Cindex >
  class I2C : public ComInterface
  {
    public:
      using CallbackT = ComInterface::CallbackT;
      /**
       * @brief Get the I2C instance
       * 
       * @return I2C& Reference to the I2C instance
       */
      static I2C& getInstance()
      {
        static I2C i2c;
        ensureI2CLink = true;
        return i2c;
      }

      /**
       * @brief Configure the I2C interface with
       * the specified config
       * 
       * @param config I2C config
       */
      template<typename I2CconfigT>
      void configure(const I2CconfigT& config)
      {
        // store the slave address
        slaveAddress_ = config.slaveAddress;

        i2cInstance_->CCR &= ~(0xFFF << I2C_CCR_CCR_Pos);
        i2cInstance_->TRISE &= ~(0x3F << I2C_TRISE_TRISE_Pos);
        if constexpr(config.clockFrequency <= 100000U)
        {
          // standard mode
          i2cInstance_->CCR &= ~I2C_CCR_FS;
          i2cInstance_->CCR |= (Pclk1Frequency / (2 * config.clockFrequency)) << I2C_CCR_CCR_Pos;
          // configure rise time register (for sm, 1000ns)
          i2cInstance_->TRISE |= 43 << I2C_TRISE_TRISE_Pos;
        }
        else
        {
          // fast mode
          i2cInstance_->CCR |= I2C_CCR_FS;
          i2cInstance_->CCR &= ~I2C_CCR_DUTY;
          i2cInstance_->CCR |= (Pclk1Frequency / (3 * config.clockFrequency)) << I2C_CCR_CCR_Pos;
          // configure rise time register (for fm, 300ns)
          i2cInstance_->TRISE |= 13 << I2C_TRISE_TRISE;
        }
        using SdaPinT = typename I2CconfigT::sdaPin;
        using SclPinT = typename I2CconfigT::sclPin;
        static_assert(checkPins<SdaPinT, SclPinT>(), "Invalid I2C pins");
        SdaPinT sda;
        SclPinT scl;
        sda.setOutputType(GpioOutputType::OpenDrain);
        scl.setOutputType(GpioOutputType::OpenDrain);
        if constexpr(I2Cindex == 1)
        {
          sda.setAlternateFunction(SdaPinT::AlternateFunctions::I2C1_SDA);
          scl.setAlternateFunction(SclPinT::AlternateFunctions::I2C1_SCL);
        }
        else if constexpr(I2Cindex == 2)
        {
          sda.setAlternateFunction(SdaPinT::AlternateFunctions::I2C2_SDA);
          scl.setAlternateFunction(SclPinT::AlternateFunctions::I2C2_SCL);
        }
        else if constexpr(I2Cindex == 3)
        {
          sda.setAlternateFunction(SdaPinT::AlternateFunctions::I2C3_SDA);
          scl.setAlternateFunction(SclPinT::AlternateFunctions::I2C3_SCL);
        }
      }

      void setErrorCallback(const CallbackT& callback) final
      {
        errorCallback_ = callback;
      }

      bool asyncRead(std::byte * const buffer, const size_t bytesToRead, size_t& actualRead
                      , const CallbackT& callback) final
      {
        // by default, generate stop condition
        static constexpr bool doStop = true;
        return asyncRead(buffer, bytesToRead, actualRead, callback, doStop);
      }

      bool asyncRead(std::byte * const buffer, const size_t bytesToRead, size_t& actualRead
                      , const CallbackT& callback, const bool doStop)
      {
        static size_t iByte = 0;
        if(transferInProgress_) return false;
        iByte = 0;
        ///@todo Check for errors and set actualRead
        transferCallback_ = [&, bytesToRead, doStop](){
          if(i2cInstance_->SR1 & I2C_SR1_SB)
          {
            // start condition generated, send slave address next
            i2cInstance_->DR = (slaveAddress_ << 1) | 1;
          }
          else if(i2cInstance_->SR1 & I2C_SR1_ADDR || i2cInstance_->SR1 & I2C_SR1_RXNE)
          {
            if(iByte == bytesToRead - 1)
            {
              buffer[iByte++] = static_cast<std::byte>(i2cInstance_->DR);
            }
            else if(iByte == bytesToRead)
            {
              // no more data left, generate stop condition if requested
              if(doStop)
              {
                i2cInstance_->CR1 |= I2C_CR1_STOP;
              }
              actualRead = iByte;
              // call user callback
              callback();
              // disable I2C
              disable();
            }
            else
            {
              buffer[iByte++] = static_cast<std::byte>(i2cInstance_->DR);
            }
          }
        };
        // turn on ACK generation only if more than one byte
        // needs to be read
        if(bytesToRead == 1)
        {
          i2cInstance_->SR1 &= ~I2C_CR1_ACK;
        }
        else
        {
          i2cInstance_->CR1 |= I2C_CR1_ACK;
        }
        // enable peripheral
        enable();
        // generate start condition
        i2cInstance_->CR1 |= I2C_CR1_START;
        transferInProgress_ = true;
        return true;
      }

      bool asyncWrite(const std::byte * const buffer, const size_t bytesToWrite, size_t& actualWrite
                      , const CallbackT& callback) final
      {
        // by default, generate stop condition
        static constexpr bool doStop = true;
        return asyncWrite(buffer, bytesToWrite, actualWrite, callback, doStop);
      }

      bool asyncWrite(const std::byte * const buffer, const size_t bytesToWrite, size_t& actualWrite
                      , const CallbackT& callback, const bool doStop)
      {
        static size_t iByte = 0;
        if(transferInProgress_) return false;
        iByte = 0;
        ///@todo Check for errors and set actualWrite
        transferCallback_ = [&, bytesToWrite, doStop](){
          if(i2cInstance_->SR1 & I2C_SR1_SB)
          {
            // start condition generated, send slave address next
            i2cInstance_->DR = (slaveAddress_ << 1);
          }
          else if(i2cInstance_->SR1 & I2C_SR1_ADDR || i2cInstance_->SR1 & I2C_SR1_TXE)
          {
            (void)i2cInstance_->SR2;
            if(iByte == bytesToWrite)
            {
              // no more data left, generate stop condition if requested
              if(doStop)
              {
                i2cInstance_->CR1 |= I2C_CR1_STOP;
              }
              actualWrite = iByte;
              // call user callback
              callback();
              // disable I2C
              disable();
            }
            else
            {
              i2cInstance_->DR = static_cast<uint8_t>(buffer[iByte++]);
            }
          }
        };
        // enable peripheral
        enable();
        // generate start condition
        i2cInstance_->CR1 |= I2C_CR1_START;
        transferInProgress_ = true;
        return true;
      }

      /**
       * Enable the I2C peripheral
       * 
       */
      void enable()
      {
        i2cInstance_->CR1 |= I2C_CR1_PE;
      }

      /**
       * Disable the I2C peripheral
       * 
       */
      void disable()
      {
        i2cInstance_->CR1 &= ~I2C_CR1_PE;
      }
    private:
      I2C()
      {
        static_assert(I2Cindex == 1 || I2Cindex == 2 || I2Cindex == 3, "Invalid I2C instance");
        IRQn_Type eventIrqNumber;
        IRQn_Type errorIrqNumber;
        if constexpr(I2Cindex == 1)
        {
          RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
          eventIrqNumber = I2C1_EV_IRQn;
          errorIrqNumber = I2C1_ER_IRQn;
        }
        else if constexpr(I2Cindex == 2)
        {
          RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
          eventIrqNumber = I2C2_EV_IRQn;
          errorIrqNumber = I2C2_ER_IRQn;
        }
        else if constexpr(I2Cindex == 3)
        {
          RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
          eventIrqNumber = I2C3_EV_IRQn;
          errorIrqNumber = I2C3_ER_IRQn;
        }
        i2cInstance_->CR2 &= ~(0x3F << I2C_CR2_FREQ_Pos);
        i2cInstance_->CR2 |= (Pclk1Frequency / 1000000U) << I2C_CR2_FREQ_Pos;
        i2cInstance_->FLTR &= ~(0xF << I2C_FLTR_DNF_Pos);
        i2cInstance_->FLTR |= 0xF << I2C_FLTR_DNF_Pos;
        i2cInstance_->CR2 |= I2C_CR2_ITBUFEN;
        i2cInstance_->CR2 |= I2C_CR2_ITEVTEN;
        i2cInstance_->CR2 |= I2C_CR2_ITERREN;
        NVIC_ClearPendingIRQ(eventIrqNumber);
        NVIC_EnableIRQ(eventIrqNumber);
        NVIC_ClearPendingIRQ(errorIrqNumber);
        NVIC_EnableIRQ(errorIrqNumber);
      }
      
      // singleton class
      I2C(const I2C&) = delete;
      I2C(I2C&&) = delete;
      void operator=(const I2C&) = delete;
      void operator=(I2C&&) = delete;

      static constexpr I2C_TypeDef *getI2Cinstance()
      {
        switch(I2Cindex)
        {
          case 1:
            return I2C1;
            break;
          case 2:
            return I2C2;
            break;
          case 3:
            return I2C3;
            break;
          default:
            return nullptr;
            break;
        }
      }

      template<typename PinSdaT, typename PinSclT>
      static constexpr bool checkPins()
      {
        if constexpr (I2Cindex == 1)
        {
          constexpr bool sdaCorrect = gpioCheckAlternateFunction<PinSdaT::pinPort, PinSdaT::pinNumber>(GpioAlternateFunctionId::I2C1_SDA);
          constexpr bool sclCorrect = gpioCheckAlternateFunction<PinSclT::pinPort, PinSclT::pinNumber>(GpioAlternateFunctionId::I2C1_SCL);
          return (sdaCorrect && sclCorrect);
        }
        else if constexpr (I2Cindex == 2)
        {
          constexpr bool sdaCorrect = 
                  gpioCheckAlternateFunction<PinSdaT::pinPort, PinSdaT::pinNumber>(GpioAlternateFunctionId::I2C2_SDA_AF4)
                  || gpioCheckAlternateFunction<PinSdaT::pinPort, PinSdaT::pinNumber>(GpioAlternateFunctionId::I2C2_SDA_AF9);
          constexpr bool sclCorrect = gpioCheckAlternateFunction<PinSclT::pinPort, PinSclT::pinNumber>(GpioAlternateFunctionId::I2C2_SCL);
          return (sdaCorrect && sclCorrect);
        }
        else if constexpr (I2Cindex == 3)
        {
          constexpr bool sdaCorrect = 
                  gpioCheckAlternateFunction<PinSdaT::pinPort, PinSdaT::pinNumber>(GpioAlternateFunctionId::I2C3_SDA_AF4)
                  || gpioCheckAlternateFunction<PinSdaT::pinPort, PinSdaT::pinNumber>(GpioAlternateFunctionId::I2C3_SDA_AF9);
          constexpr bool sclCorrect = gpioCheckAlternateFunction<PinSclT::pinPort, PinSclT::pinNumber>(GpioAlternateFunctionId::I2C3_SCL);
          return (sdaCorrect && sclCorrect);
        }
        else
        {
          return false;
        }

      }

      I2C_TypeDef *const i2cInstance_ = getI2Cinstance();
      uint8_t slaveAddress_;
      volatile bool transferInProgress_ = false;
    public:
      inline static CallbackT transferCallback_;
      inline static CallbackT errorCallback_;
  };
}


#endif //STM32_I2C_HPP