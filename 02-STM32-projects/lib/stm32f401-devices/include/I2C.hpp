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
  
  /**
   * Enumeration holding the possible I2C errors.
   * Can be used for checking different error conditions.
   */
  enum class I2Cerror
  {
    NoError,
    BusError,
    AcknowledgeFailure,
    ArbitrationLost,
    OverrunUnderrunError
  };

  /**
   * @brief Type holding required information
   * for configuring the MCU I2C interface
   * as master to communicate with a slave
   * 
   * @tparam ClockFrequency The I2C clock frequency
   * @tparam SlaveAddress The 7-bit address of the slave
   * @tparam SdaPin SDA pin
   * @tparam SclPin SCL pin
   */
  template <
    uint32_t ClockFrequency
    , uint8_t SlaveAddress
    , typename SdaPin
    , typename SclPin>
  struct I2Cconfig
  {
    static constexpr uint32_t clockFrequency = ClockFrequency;
    static constexpr uint8_t slaveAddress = SlaveAddress;
    using sdaPin = SdaPin;
    using sclPin = SclPin;
  };

  /**
   * @brief Interface for I2C peripheral.
   * Currently supports master mode only.
   * 
   * @tparam I2Cindex The number of the I2C instance.
   */
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
      void configure(const I2CconfigT& config = I2CconfigT())
      {
        /** @note The I2C module on this MCU locks with
         * BUSY = 1 if the pins are not configured before enabling
         * clock for the interface. The internal pull-ups also
         * seem to prevent this condition from happening.
         */
        // turn peripheral off
        disable();
        // turn off clock for the peripheral
        if constexpr(I2Cindex == 1)
        {
          RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
        }
        else if constexpr(I2Cindex == 2)
        {
          RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
        }
        else if constexpr(I2Cindex == 3)
        {
          RCC->APB1ENR &= ~RCC_APB1ENR_I2C3EN;
        }
        // configure pins
        using SdaPinT = typename I2CconfigT::sdaPin;
        using SclPinT = typename I2CconfigT::sclPin;
        configurePins<SdaPinT, SclPinT>();

        // turn on the clock and reset the peripheral
        if constexpr(I2Cindex == 1)
        {
          RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        }
        else if constexpr(I2Cindex == 2)
        {
          RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        }
        else if constexpr(I2Cindex == 3)
        {
          RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
        }  
        reset();
        // basic configuration
        i2cInstance_->FLTR |= I2C_FLTR_ANOFF;  //< analog filter may be problematic, so turn it off
        i2cInstance_->CR2 &= ~(0x3F << I2C_CR2_FREQ_Pos);
        i2cInstance_->CR2 |= (Pclk1Frequency / 1000000U) << I2C_CR2_FREQ_Pos;
        i2cInstance_->FLTR &= ~(0xF << I2C_FLTR_DNF_Pos);
        i2cInstance_->FLTR |= 0xF << I2C_FLTR_DNF_Pos;
        i2cInstance_->CR2 |= I2C_CR2_ITBUFEN;
        i2cInstance_->CR2 |= I2C_CR2_ITEVTEN;
        i2cInstance_->CR2 |= I2C_CR2_ITERREN;

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
      }

      void setErrorCallback(const CallbackT& callback) final
      {
        userErrorCallback_ = callback;
      }

      bool asyncRead(std::byte * const buffer, const size_t bytesToRead, size_t& actualRead
                      , const CallbackT& callback) final
      {
        static size_t iByte = 0;
        if(transferInProgress_) return false;
        userTransferCallback_ = callback;
        iByte = 0;
        transferCallback_ = [this, buffer, bytesToRead, &actualRead](){
          errorStatus_ = I2Cerror::NoError;  //< reset error status
          if(i2cInstance_->SR1 & I2C_SR1_SB)
          {
            // start condition generated, send slave address next
            i2cInstance_->DR = (slaveAddress_ << 1) | 1;
          }
          else if(i2cInstance_->SR1 & I2C_SR1_ADDR)
          {
            if(bytesToRead == 1)
            {
              i2cInstance_->CR1 &= ~I2C_CR1_ACK;
            }
            (void)i2cInstance_->SR2;
            (void)i2cInstance_->DR;
          }
          else if(i2cInstance_->SR1 & I2C_SR1_RXNE)
          {
            if(iByte == bytesToRead - 1)
            {
              buffer[iByte++] = static_cast<std::byte>(i2cInstance_->DR);
              // no more data left, generate stop condition
              i2cInstance_->CR1 |= I2C_CR1_STOP;
              actualRead = iByte;
              // call user callback
              if(userTransferCallback_) userTransferCallback_();
              transferInProgress_ = false;
              // disable I2C
              disable();
            }
            else
            {
              buffer[iByte++] = static_cast<std::byte>(i2cInstance_->DR);
              actualRead = iByte;
              if(iByte == bytesToRead - 1)
              {
                // turn off ACK
                i2cInstance_->CR1 &= ~I2C_CR1_ACK;
              }
            }
          }
        };
        // enable peripheral
        enable();
        // turn on ACK generation
        i2cInstance_->CR1 |= I2C_CR1_ACK;
        // generate start condition
        i2cInstance_->CR1 |= I2C_CR1_START;
        transferInProgress_ = true;
        return true;
      }

      bool asyncWrite(const std::byte * const buffer, const size_t bytesToWrite, size_t& actualWrite
                      , const CallbackT& callback) final
      {
        static size_t iByte = 0;
        if(transferInProgress_) return false;
        userTransferCallback_ = callback;
        iByte = 0;
        transferCallback_ = [this, buffer, bytesToWrite, &actualWrite](){
          errorStatus_ = I2Cerror::NoError;  //< reset error status
          if(i2cInstance_->SR1 & I2C_SR1_SB)
          {
            // start condition generated, send slave address next
            i2cInstance_->DR = (slaveAddress_ << 1);
          }
          else if(i2cInstance_->SR1 & I2C_SR1_ADDR)
          {
            (void)i2cInstance_->SR2;
            i2cInstance_->DR = static_cast<uint8_t>(buffer[iByte++]);
          }
          else if(i2cInstance_->SR1 & I2C_SR1_TXE)
          {
            if(iByte == bytesToWrite)
            {
              // no more data left, generate stop condition if requested
              i2cInstance_->CR1 |= I2C_CR1_STOP;
              actualWrite = iByte;
              // call user callback
              if(userTransferCallback_) userTransferCallback_();
              transferInProgress_ = false;
              // disable I2C
              disable();
            }
            else
            {
              i2cInstance_->DR = static_cast<uint8_t>(buffer[iByte++]);
              actualWrite = iByte;
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
       * @brief Write the bytes from the user-defined buffer
       * via the communication interface. Before writing the data,
       * write control bytes or memory address bytes.
       * After write is done, call the user-passed callback function.
       * @note This function is non-blocking.
       * 
       * @param buffer The buffer which holds the bytes to be written.
       * @param bytesToWrite The number of bytes to write.
       * @param controlOrMemAddress The buffer which holds the control bytes
       * or the memory address bytes.
       * @param controlOrMemAddressSize The size of the buffer holding the
       * control bytes or memory address bytes.
       * @param actualWrite The actual number of bytes that were written.
       * @param callback The callback function.
       * @return true The write operation was started successfully.
       * @return false There is a transfer in progress.
       */
      bool asyncMemWrite(const std::byte * const buffer, const size_t bytesToWrite
                        , const std::byte * const controlOrMemAddress, const size_t controlOrMemAddressSize
                        , size_t& actualWrite, const CallbackT& callback)
      {
        static size_t iByte = 0;
        if(transferInProgress_) return false;
        userTransferCallback_ = callback;
        iByte = 0;
        transferCallback_ = [this, controlOrMemAddress, controlOrMemAddressSize
                            , buffer, bytesToWrite, &actualWrite](){
          errorStatus_ = I2Cerror::NoError;  //< reset error status
          if(i2cInstance_->SR1 & I2C_SR1_SB)
          {
            // start condition generated, send slave address next
            i2cInstance_->DR = (slaveAddress_ << 1);
          }
          else if(i2cInstance_->SR1 & I2C_SR1_ADDR)
          {
            (void)i2cInstance_->SR2;
            i2cInstance_->DR = static_cast<uint8_t>(controlOrMemAddress[iByte++]);
          }
          else if(i2cInstance_->SR1 & I2C_SR1_TXE)
          {
            if(iByte == bytesToWrite + controlOrMemAddressSize)
            {
              // no more data left, generate stop condition if requested
              i2cInstance_->CR1 |= I2C_CR1_STOP;
              actualWrite = iByte;
              // call user callback
              if(userTransferCallback_) userTransferCallback_();
              transferInProgress_ = false;
              // disable I2C
              disable();
            }
            else
            {
              if(iByte >= controlOrMemAddressSize)
              {
                i2cInstance_->DR = static_cast<uint8_t>(buffer[iByte++ - controlOrMemAddressSize]);
              }
              else
              {
                i2cInstance_->DR = static_cast<uint8_t>(controlOrMemAddress[iByte++]);
              }
              actualWrite = iByte;
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
       * @brief Write the bytes from the user-defined buffer, but
       * first send control or memory address bytes.
       * @note This function is blocking.
       * 
       * @param buffer The buffer which holds the bytes to be written.
       * @param bytesToWrite The number of bytes to write.
       * @param controlOrMemAddress The buffer which holds the control bytes
       * or the memory address bytes.
       * @param controlOrMemAddressSize The size of the buffer holding the
       * control bytes or memory address bytes.
       * @return size_t The actual number of bytes that were written.
       */
      size_t memWrite(const std::byte * const buffer, const size_t bytesToWrite
                  , const std::byte * const controlOrMemAddress, const size_t controlOrMemAddressSize)
      {
        size_t writeSize = 0;
        volatile bool writeDone = false;
        if(!asyncMemWrite(buffer, bytesToWrite, controlOrMemAddress, controlOrMemAddressSize
                        , writeSize, [&writeDone](){ writeDone = true; }))
        {
          return 0;
        }
        while(!writeDone);
        return writeSize;
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

      /**
       * Reset the I2C peripheral
       * 
       */
      void reset()
      {
        i2cInstance_->CR1 |= I2C_CR1_SWRST;
        i2cInstance_->CR1 &= ~I2C_CR1_SWRST;
        transferCallback_ = nullptr;
        errorStatus_ = I2Cerror::NoError;
      }

      /**
       * @brief Get the error status value.
       * 
       * @return I2Cerror The I2C error.
       */
      I2Cerror getErrorStatus()
      {
        return errorStatus_;
      }
    private:
      I2C() : errorStatus_(I2Cerror::NoError)
      {
        static_assert(I2Cindex == 1 || I2Cindex == 2 || I2Cindex == 3, "Invalid I2C instance");
        IRQn_Type eventIrqNumber;
        IRQn_Type errorIrqNumber;
        if constexpr(I2Cindex == 1)
        {
          eventIrqNumber = I2C1_EV_IRQn;
          errorIrqNumber = I2C1_ER_IRQn;
        }
        else if constexpr(I2Cindex == 2)
        {
          eventIrqNumber = I2C2_EV_IRQn;
          errorIrqNumber = I2C2_ER_IRQn;
        }
        else if constexpr(I2Cindex == 3)
        {
          eventIrqNumber = I2C3_EV_IRQn;
          errorIrqNumber = I2C3_ER_IRQn;
        }
        // set error callback, which updates error status
        // and calls user provided error callback
        errorCallback_ = [this]() {
          // set error status
          if (i2cInstance_->SR1 & I2C_SR1_AF)
          {
            errorStatus_ = I2Cerror::AcknowledgeFailure;
          }
          else if (i2cInstance_->SR1 & I2C_SR1_BERR)
          {
            errorStatus_ = I2Cerror::BusError;
          }
          else if (i2cInstance_->SR1 & I2C_SR1_OVR)
          {
            errorStatus_ = I2Cerror::OverrunUnderrunError;
          }
          else if (i2cInstance_->SR1 & I2C_SR1_ARLO)
          {
            errorStatus_ = I2Cerror::ArbitrationLost;
          }
          // disable I2C peripheral
          disable();
          // call user error callback
          if(userErrorCallback_)
          {
            userErrorCallback_();
          }
          // erase transfer callback
          transferCallback_ = nullptr;
        };
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
      static constexpr void configurePins()
      {
        PinSdaT sda;
        PinSclT scl;
        sda.setOutputType(GpioOutputType::OpenDrain);
        scl.setOutputType(GpioOutputType::OpenDrain);
        sda.setPullType(GpioPullType::PullUp);
        scl.setPullType(GpioPullType::PullUp);
        if constexpr (I2Cindex == 1)
        {
          constexpr bool sdaCorrect = gpioCheckAlternateFunction<PinSdaT::pinPort, PinSdaT::pinNumber>(GpioAlternateFunctionId::I2C1_SDA);
          constexpr bool sclCorrect = gpioCheckAlternateFunction<PinSclT::pinPort, PinSclT::pinNumber>(GpioAlternateFunctionId::I2C1_SCL);
          static_assert(sdaCorrect && sclCorrect, "Invalid I2C pins");
          sda.setAlternateFunction(PinSdaT::AlternateFunctions::I2C1_SDA);
          scl.setAlternateFunction(PinSclT::AlternateFunctions::I2C1_SCL);
        }
        else if constexpr (I2Cindex == 2)
        {
          constexpr bool sdaCorrect = 
                  gpioCheckAlternateFunction<PinSdaT::pinPort, PinSdaT::pinNumber>(GpioAlternateFunctionId::I2C2_SDA_AF4)
                  || gpioCheckAlternateFunction<PinSdaT::pinPort, PinSdaT::pinNumber>(GpioAlternateFunctionId::I2C2_SDA_AF9);
          constexpr bool sclCorrect = gpioCheckAlternateFunction<PinSclT::pinPort, PinSclT::pinNumber>(GpioAlternateFunctionId::I2C2_SCL);
          static_assert(sdaCorrect && sclCorrect, "Invalid I2C pins");
          sda.setAlternateFunction(PinSdaT::AlternateFunctions::I2C2_SDA);
          scl.setAlternateFunction(PinSclT::AlternateFunctions::I2C2_SCL);
        }
        else if constexpr (I2Cindex == 3)
        {
          constexpr bool sdaCorrect = 
                  gpioCheckAlternateFunction<PinSdaT::pinPort, PinSdaT::pinNumber>(GpioAlternateFunctionId::I2C3_SDA_AF4)
                  || gpioCheckAlternateFunction<PinSdaT::pinPort, PinSdaT::pinNumber>(GpioAlternateFunctionId::I2C3_SDA_AF9);
          constexpr bool sclCorrect = gpioCheckAlternateFunction<PinSclT::pinPort, PinSclT::pinNumber>(GpioAlternateFunctionId::I2C3_SCL);
          static_assert(sdaCorrect && sclCorrect, "Invalid I2C pins");
          sda.setAlternateFunction(PinSdaT::AlternateFunctions::I2C3_SDA);
          scl.setAlternateFunction(PinSclT::AlternateFunctions::I2C3_SCL);
        }
      }

      I2C_TypeDef *const i2cInstance_ = getI2Cinstance();
      uint8_t slaveAddress_;
      volatile bool transferInProgress_ = false;
      volatile I2Cerror errorStatus_;
      CallbackT userTransferCallback_;
      CallbackT userErrorCallback_;
    public:
      inline static CallbackT transferCallback_;
      inline static CallbackT errorCallback_;
  };
}


#endif //STM32_I2C_HPP