#ifndef STM32_SPI_HPP
#define STM32_SPI_HPP

#include <stm32f4xx.h>
#include <cstdint>
#include <ratio>
#include <functional>

#include "Clock.hpp"
#include "ComInterface.hpp"
#include "Gpio.hpp"

namespace Stm32
{
  extern bool ensureSpiLink;  //< ensure that the source file with the IRQ handlers is linked

  /**
   * Enumeration holding the possible SPI errors.
   * Can be used for checking different error conditions.
   */
  enum class SPIerror
  {
    NoError,
    ModeFaultError,
    OverrunError,
    FrameFormatError,
  };

  enum class SpiMode : uint8_t
  {
    Mode0  = 0,
    Mode1  = 1,
    Mode2  = 2,
    Mode3  = 3
  };

  template<
      typename MosiPinT
    , typename MisoPinT
    , typename SckPinT
    , typename SsPinT>
  struct SpiPins
  {
    using mosiPin = MosiPinT;
    using misoPin = MisoPinT;
    using sckPin = SckPinT;
    using ssPin = SsPinT;
  };

  template<
      uint32_t ClockFrequency
    , SpiMode Mode
    , bool FrameSize16Bits
    , bool LSBfirst
    , typename PinsT
    , bool HardwareSs>
  struct SpiConfig
  {
    static constexpr uint32_t clockFrequency = ClockFrequency;
    static constexpr bool hardwareSs = HardwareSs;
    static constexpr auto spiMode = Mode;
    static constexpr bool frameSize16Bits = FrameSize16Bits;
    static constexpr bool lsbFirst = LSBfirst;
    using pinsT = PinsT;
  };

  template < uint8_t SpiIndex>
  class SPI : public ComInterface
  {
    public:
      using CallbackT = ComInterface::CallbackT;

      void setErrorCallback(const CallbackT &callback)
      {
        
      }

      static SPI& getInstance()
      {
        static SPI spi;
        ensureSpiLink = true;
        return spi;
      }

      template<typename SPIconfigT>
      void configure(const SPIconfigT& config = SPIconfigT())
      {
        auto log2 = [](uint32_t n) constexpr {
          size_t res = 0;
          for(; n > 1; ++res, n/=2);
          return res;
        };

        auto getPclk = [](const uint8_t index) constexpr {
          uint32_t freq = 0;
          if (index == 1 || index == 4) freq = Pclk2Frequency;
          else if (index == 2 || index == 3) freq = Pclk1Frequency;
          return freq;
        };

        // set baud rate
        static constexpr auto pclk = getPclk(SpiIndex);
        static constexpr auto f = config.clockFrequency;        
        static constexpr auto quotient = pclk/f;
        static constexpr auto reminder = pclk - f*quotient;
        static constexpr auto roundedQuotient = quotient + std::ratio_greater_v<
                                                      std::ratio<reminder, 1>
                                                  ,   std::ratio<f, 2>>;
        static constexpr auto br = log2(roundedQuotient) - 1;
        spiInstance_->CR1 &= ~SPI_CR1_BR;
        spiInstance_->CR1 |= br << SPI_CR1_BR_Pos;

        // set SPI mode
        if constexpr(config.spiMode == SpiMode::Mode0)
        {
          spiInstance_->CR1 &= ~SPI_CR1_CPHA;
          spiInstance_->CR1 &= ~SPI_CR1_CPOL;
        }
        else if constexpr(config.spiMode == SpiMode::Mode1)
        {
          spiInstance_->CR1 |= SPI_CR1_CPHA;
          spiInstance_->CR1 &= ~SPI_CR1_CPOL;
        }
        else if constexpr(config.spiMode == SpiMode::Mode2)
        {
          spiInstance_->CR1 &= ~SPI_CR1_CPHA;
          spiInstance_->CR1 |= SPI_CR1_CPOL;
        }
        else if constexpr(config.spiMode == SpiMode::Mode3)
        {
          spiInstance_->CR1 |= SPI_CR1_CPHA;
          spiInstance_->CR1 |= SPI_CR1_CPOL;
        }

        spiInstance_->CR1 |= SPI_CR1_MSTR;  //< configure as master

        // set frame size
        frameSize16Bits_ = config.frameSize16Bits;
        if constexpr(config.frameSize16Bits)
        {
          spiInstance_->CR1 |= SPI_CR1_DFF;  //< 16-bit frame size
        }
        else
        {
          spiInstance_->CR1 &= ~SPI_CR1_DFF;  //< 8-bit frame size
        }

        if constexpr(config.lsbFirst)
        {
          spiInstance_->CR1 |= SPI_CR1_LSBFIRST;
        }
        else
        {
          spiInstance_->CR1 &= ~SPI_CR1_LSBFIRST;
        }

        // check and configure pins
        configurePins<typename SPIconfigT::pinsT>(config.hardwareSs);

        // hardware ss
        if(config.hardwareSs)
        {
          spiInstance_->CR1 &= ~SPI_CR1_SSM;
          spiInstance_->CR2 |= SPI_CR2_SSOE;
          setSsLevel_ = nullptr;
        }
        else
        {
          spiInstance_->CR1 |= SPI_CR1_SSM;
          spiInstance_->CR1 |= SPI_CR1_SSI;
          setSsLevel_ = [](const bool level){
            using PinsT = typename SPIconfigT::pinsT;
            using SsPinT = typename PinsT::ssPin;
            SsPinT ss;
            ss.setLevel(level);
          };
        }
      }

      bool asyncTransfer(std::byte * const bufferRead, const CallbackT& callbackRead
                      , const std::byte * const bufferWrite, const CallbackT& callbackWrite
                      , const size_t bytesToTransfer, size_t& actualTransfer)
      {
        static size_t iRead = 0, iWrite = 0;
        if(transferInProgress_) return false;
        iRead = iWrite = 0;
        transferCallback_ = [this, bufferRead, callbackRead, bufferWrite, callbackWrite, bytesToTransfer, &actualTransfer]()
        {
          /// @todo error checking
          if((spiInstance_->CR2 & SPI_CR2_TXEIE) && (spiInstance_->SR & SPI_SR_TXE))
          {
            if(iWrite < bytesToTransfer)
            {
              if(frameSize16Bits_)
              {
                spiInstance_->DR = *reinterpret_cast<const uint16_t*>(bufferWrite + iWrite);
                iWrite += sizeof(uint16_t);
              }
              else
              {
                spiInstance_->DR = static_cast<uint8_t>(bufferWrite[iWrite++]);
              }
              actualTransfer = iWrite;
            }
            else if(iWrite == bytesToTransfer)
            {
              disableTxIrq();
              if(callbackWrite) callbackWrite();
            }
          }
          if((spiInstance_->CR2 & SPI_CR2_RXNEIE) && (spiInstance_->SR & SPI_SR_RXNE))
          {
            if(iRead < bytesToTransfer)
            {
              if(frameSize16Bits_)
              {
                *reinterpret_cast<uint16_t*>(bufferRead + iRead) = static_cast<uint16_t>(spiInstance_->DR);
                iRead += sizeof(uint16_t);
              }
              else
              {
                bufferRead[iRead++] = static_cast<std::byte>(spiInstance_->DR);
              }
              // actualTransfer = iRead;
            }
            else if(iRead == bytesToTransfer)
            {
              disableRxIrq();
              if(callbackRead) callbackRead();
            }
          }
          
          bool writeFinished = (bufferWrite != nullptr && iWrite == bytesToTransfer) || (bufferWrite == nullptr);
          bool readFinished = (bufferRead != nullptr && iRead == bytesToTransfer) || (bufferRead == nullptr);
          if(writeFinished && readFinished)
          {
            // if using software SS
            if(setSsLevel_)
            {
              // set SS high
              setSsLevel_(true);
            }
            disable();
            transferInProgress_ = false;
          }
        };
        
        // start clock
        enable();
        // if using software SS
        if(setSsLevel_)
        {
          // set SS low
          setSsLevel_(false);
        }
        // send initial data to kick-start transfer
        if(bufferWrite != nullptr)
        {
          if(frameSize16Bits_)
          {
            spiInstance_->DR = *reinterpret_cast<const uint16_t*>(bufferWrite + iWrite);
            iWrite += sizeof(uint16_t);
          }
          else
          {
            spiInstance_->DR = static_cast<uint8_t>(bufferWrite[iWrite++]);
          }
          actualTransfer = iWrite;
        }
        transferInProgress_ = true;
        if(bufferRead != nullptr)
        {
          enableRxIrq();
        }

        if(bufferWrite != nullptr)
        {
          enableTxIrq();
        }
        return true;                
      }

      size_t transfer(std::byte * const bufferRead, const std::byte * const bufferWrite, const size_t bytesToTransfer)
      {
        size_t bytesTransfered = 0;
        if(!asyncTransfer(bufferRead, nullptr
                        , bufferWrite, nullptr
                        , bytesToTransfer, bytesTransfered))
        {
          return 0;
        }
        while(transferInProgress_)
        {
          /// @todo upgrade for errors
        }
        return bytesTransfered;
      }

      bool asyncRead(std::byte * const buffer, const size_t bytesToRead, size_t& actualRead, const CallbackT& callback) final
      {
        return asyncTransfer(buffer, callback, nullptr, {}, bytesToRead, actualRead);
      }

      bool asyncWrite(const std::byte * const buffer, const size_t bytesToWrite, size_t& actualWrite, const CallbackT& callback) final
      {
        return asyncTransfer(nullptr, {}, buffer, callback, bytesToWrite, actualWrite);
      }

      void enable()
      {
        spiInstance_->CR1 |= SPI_CR1_SPE;
      }

      void disable()
      {
        spiInstance_->CR1 &= ~SPI_CR1_SPE;
      }

    private:
      SPI()
      {
        static_assert(SpiIndex >= 1 && SpiIndex <= 4, "Invalid SPI instance");
        
        // turn on clock for the peripheral
        if constexpr(SpiIndex == 1)
        {
          RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        }
        else if constexpr(SpiIndex == 2)
        {
          RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        }
        else if constexpr(SpiIndex == 3)
        {
          RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
        }
        else if constexpr(SpiIndex == 4)
        {
          RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
        }

        // enable interrupts
        IRQn_Type irqn;
        if constexpr(SpiIndex == 1)
        {
          irqn = SPI1_IRQn;
        }
        else if constexpr(SpiIndex == 2)
        {
          irqn = SPI2_IRQn;
        }
        else if constexpr(SpiIndex == 3)
        {
          irqn = SPI3_IRQn;
        }
        else if constexpr(SpiIndex == 4)
        {
          irqn = SPI4_IRQn;
        }
        NVIC_ClearPendingIRQ(irqn);
        NVIC_EnableIRQ(irqn);
        // spiInstance_->CR2 |= SPI_CR2_ERRIE; //< error interrupt @todo
      }
      // singleton class
      SPI(const SPI&) = delete;
      SPI(SPI&&) = delete;
      void operator=(const SPI&) = delete;
      void operator=(SPI&&) = delete;

      void enableTxIrq()
      {
        spiInstance_->CR2 |= SPI_CR2_TXEIE;   //< Tx register empty
      }

      void disableTxIrq()
      {
        spiInstance_->CR2 &= ~SPI_CR2_TXEIE;   //< Tx register empty
      }

      void enableRxIrq()
      {
        spiInstance_->CR2 |= SPI_CR2_RXNEIE;  //< Rx register not empty
      }

      void disableRxIrq()
      {
        spiInstance_->CR2 &= ~SPI_CR2_RXNEIE;  //< Rx register not empty
      }

      template<typename PinsT>
      static constexpr void configurePins(const bool hardwareSs)
      {
        using MosiPinT = typename PinsT::mosiPin;
        using MisoPinT = typename PinsT::misoPin;
        using SckPinT = typename PinsT::sckPin;
        using SsPinT = typename PinsT::ssPin;
        MosiPinT mosi;
        MisoPinT miso;
        SckPinT sck;
        SsPinT ss;
        mosi.setOutputType(GpioOutputType::PushPull);
        miso.setOutputType(GpioOutputType::PushPull);
        sck.setOutputType(GpioOutputType::PushPull);
        ss.setOutputType(GpioOutputType::PushPull);
        mosi.setSpeed(GpioSpeed::High);
        miso.setSpeed(GpioSpeed::High);
        sck.setSpeed(GpioSpeed::High);
        ss.setSpeed(GpioSpeed::High);
       
        if constexpr (SpiIndex == 1)
        {
          constexpr bool mosiCorrect = gpioCheckAlternateFunction<MosiPinT::pinPort, MosiPinT::pinNumber>(GpioAlternateFunctionId::SPI1_MOSI);
          constexpr bool misoCorrect = gpioCheckAlternateFunction<MisoPinT::pinPort, MisoPinT::pinNumber>(GpioAlternateFunctionId::SPI1_MISO);
          constexpr bool sckCorrect = gpioCheckAlternateFunction<SckPinT::pinPort, SckPinT::pinNumber>(GpioAlternateFunctionId::SPI1_SCK);
          constexpr bool ssCorrect = gpioCheckAlternateFunction<SsPinT::pinPort, SsPinT::pinNumber>(GpioAlternateFunctionId::SPI1_NSS);
          if(hardwareSs)
          {
            static_assert(mosiCorrect && misoCorrect && sckCorrect && ssCorrect, "Invalid SPI pins");
          }
          else
          {
            static_assert(mosiCorrect && misoCorrect && sckCorrect, "Invalid SPI pins");
          }
          mosi.setAlternateFunction(MosiPinT::AlternateFunctions::SPI1_MOSI);
          miso.setAlternateFunction(MisoPinT::AlternateFunctions::SPI1_MISO);
          sck.setAlternateFunction(SckPinT::AlternateFunctions::SPI1_SCK);
          if (hardwareSs)
          {
            ss.setAlternateFunction(SsPinT::AlternateFunctions::SPI1_NSS);
          }
          else
          {
            ss.setMode(GpioMode::Output);
          }
        }
        else if constexpr (SpiIndex == 2)
        {
          constexpr bool mosiCorrect = gpioCheckAlternateFunction<MosiPinT::pinPort, MosiPinT::pinNumber>(GpioAlternateFunctionId::SPI2_MOSI);
          constexpr bool misoCorrect = gpioCheckAlternateFunction<MisoPinT::pinPort, MisoPinT::pinNumber>(GpioAlternateFunctionId::SPI2_MISO);
          constexpr bool sckCorrect = gpioCheckAlternateFunction<SckPinT::pinPort, SckPinT::pinNumber>(GpioAlternateFunctionId::SPI2_SCK);
          constexpr bool ssCorrect = gpioCheckAlternateFunction<SsPinT::pinPort, SsPinT::pinNumber>(GpioAlternateFunctionId::SPI2_NSS);
          if(hardwareSs)
          {
            static_assert(mosiCorrect && misoCorrect && sckCorrect && ssCorrect, "Invalid SPI pins");
          }
          else
          {
            static_assert(mosiCorrect && misoCorrect && sckCorrect, "Invalid SPI pins");
          }
          mosi.setAlternateFunction(MosiPinT::AlternateFunctions::SPI2_MOSI);
          miso.setAlternateFunction(MisoPinT::AlternateFunctions::SPI2_MISO);
          sck.setAlternateFunction(SckPinT::AlternateFunctions::SPI2_SCK);
          if (hardwareSs)
          {
            ss.setAlternateFunction(SsPinT::AlternateFunctions::SPI2_NSS);
          }
          else
          {
            ss.setMode(GpioMode::Output);
          }
        }
        else if constexpr (SpiIndex == 3)
        {
          constexpr bool mosiCorrect = gpioCheckAlternateFunction<MosiPinT::pinPort, MosiPinT::pinNumber>(GpioAlternateFunctionId::SPI3_MOSI_AF5)
                                    || gpioCheckAlternateFunction<MosiPinT::pinPort, MosiPinT::pinNumber>(GpioAlternateFunctionId::SPI3_MOSI_AF6);
          constexpr bool misoCorrect = gpioCheckAlternateFunction<MisoPinT::pinPort, MisoPinT::pinNumber>(GpioAlternateFunctionId::SPI3_MISO);
          constexpr bool sckCorrect = gpioCheckAlternateFunction<SckPinT::pinPort, SckPinT::pinNumber>(GpioAlternateFunctionId::SPI3_SCK);
          constexpr bool ssCorrect = gpioCheckAlternateFunction<SsPinT::pinPort, SsPinT::pinNumber>(GpioAlternateFunctionId::SPI3_NSS);
          if(hardwareSs)
          {
            static_assert(mosiCorrect && misoCorrect && sckCorrect && ssCorrect, "Invalid SPI pins");
          }
          else
          {
            static_assert(mosiCorrect && misoCorrect && sckCorrect, "Invalid SPI pins");
          }
          mosi.setAlternateFunction(MosiPinT::AlternateFunctions::SPI3_MOSI);
          miso.setAlternateFunction(MisoPinT::AlternateFunctions::SPI3_MISO);
          sck.setAlternateFunction(SckPinT::AlternateFunctions::SPI3_SCK);
          if (hardwareSs)
          {
            ss.setAlternateFunction(SsPinT::AlternateFunctions::SPI3_NSS);
          }
          else
          {
            ss.setMode(GpioMode::Output);
          }
        }
        else if constexpr (SpiIndex == 4)
        {
          constexpr bool mosiCorrect = gpioCheckAlternateFunction<MosiPinT::pinPort, MosiPinT::pinNumber>(GpioAlternateFunctionId::SPI4_MOSI);
          constexpr bool misoCorrect = gpioCheckAlternateFunction<MisoPinT::pinPort, MisoPinT::pinNumber>(GpioAlternateFunctionId::SPI4_MISO);
          constexpr bool sckCorrect = gpioCheckAlternateFunction<SckPinT::pinPort, SckPinT::pinNumber>(GpioAlternateFunctionId::SPI4_SCK);
          constexpr bool ssCorrect = gpioCheckAlternateFunction<SsPinT::pinPort, SsPinT::pinNumber>(GpioAlternateFunctionId::SPI4_NSS);
          if(hardwareSs)
          {
            static_assert(mosiCorrect && misoCorrect && sckCorrect && ssCorrect, "Invalid SPI pins");
          }
          else
          {
            static_assert(mosiCorrect && misoCorrect && sckCorrect, "Invalid SPI pins");
          }
          mosi.setAlternateFunction(MosiPinT::AlternateFunctions::SPI4_MOSI);
          miso.setAlternateFunction(MisoPinT::AlternateFunctions::SPI4_MISO);
          sck.setAlternateFunction(SckPinT::AlternateFunctions::SPI4_SCK);
          if (hardwareSs)
          {
            ss.setAlternateFunction(SsPinT::AlternateFunctions::SPI4_NSS);
          }
          else
          {
            ss.setMode(GpioMode::Output);
          }
        }
      }

      static constexpr SPI_TypeDef* getSPIinstance()
      {
        switch(SpiIndex)
        {
          case 1:
            return SPI1;
            break;
          case 2:
            return SPI2;
            break;
          case 3:
            return SPI3;
            break;
          case 4:
            return SPI4;
            break;
          default:
            return nullptr;
            break;
        }
      }
      SPI_TypeDef *const spiInstance_ = getSPIinstance();
      volatile bool transferInProgress_ = false;
      bool frameSize16Bits_ = false;
      std::function<void(bool)> setSsLevel_ = nullptr;
    public:
      inline static CallbackT transferCallback_;
      inline static CallbackT errorCallback_;
  };
}
#endif // STM32_SPI_HPP