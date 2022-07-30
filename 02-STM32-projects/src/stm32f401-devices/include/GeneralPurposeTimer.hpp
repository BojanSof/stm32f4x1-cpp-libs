#ifndef STM32_GENERAL_PURPOSE_TIMER_HPP
#define STM32_GENERAL_PURPOSE_TIMER_HPP

#include <functional>

#include <stm32f4xx.h>
#include <Clock.hpp>

namespace Stm32
{
  extern bool ensureGeneralPurposeTimerLink; //< ensure that the source file with the IRQ handlers is linked

  enum class TimerMode
  {
      InputCapture,
      PwmInput,
      ForcedOutput,
      OutputCompare,
      PwmOutput,
      OnePulse,
      EncoderInterface
  };

  enum class TimerDirection : uint8_t
  {
      Up    = 0,
      Down  = 1
  };

  enum class TimerCenterAlignedMode : uint8_t
  {
      Edge    = 0,
      Mode1   = 1,
      Mode2   = 2,
      Mode3   = 3
  };

  template < 
        TimerMode Mode
      , TimerDirection Direction
      , TimerCenterAlignedMode CenterAlignedMode >
  struct TimerModeConfig
  {
      static constexpr auto mode = Mode;
      static constexpr auto direction  = Direction;
      static constexpr auto centerAlignedMode  = CenterAlignedMode;
  };



  template <
          uint32_t Period
        , uint32_t DutyCycle
        , uint8_t CcChannel
        , bool OutputEnable
        , bool Polarity >
  struct PwmModeConfig : public TimerModeConfig<
                                      TimerMode::PwmOutput, 
                                      TimerDirection::Up, 
                                      TimerCenterAlignedMode::Edge >
  {
    static constexpr auto period = Period;
    static constexpr auto dutyCycle = DutyCycle;
    static constexpr auto ccChannel = CcChannel;
    static constexpr auto outputEnable = OutputEnable;
    static constexpr auto polarity = Polarity;
  };

  enum class TimerDigitalFilter
  {
    NoFilter        = 0b0000,
    FreqInternalN2  = 0b0001,
    FreqInternalN4  = 0b0010,
    FreqInternalN8  = 0b0011,
    FreqDtsDiv2N6   = 0b0100,
    FreqDtsDiv2N8   = 0b0101,
    FreqDtsDiv4N6   = 0b0110,
    FreqDtsDiv4N8   = 0b0111,
    FreqDtsDiv8N6   = 0b1000,
    FreqDtsDiv8N8   = 0b1001,
    FreqDtsDiv16N5  = 0b1010,
    FreqDtsDiv16N6  = 0b1011,
    FreqDtsDiv16N8  = 0b1100,
    FreqDtsDiv32N5  = 0b1101,
    FreqDtsDiv32N6  = 0b1110,
    FreqDtsDiv32N8  = 0b1111
  };

  enum class TimerCapturePolarity
  {
    RisingEdge    = 0b000,
    FallingEdge   = 0b001,
    BothEdges     = 0b101
  };

  enum class TimerCapturePrescaler
  {
    Div1  = 0b00,
    Div2  = 0b01,
    Div4  = 0b10,
    Div8  = 0b11
  };

  enum class TimerCaptureCompareSelection : uint8_t
  {
    Output          = 0b00,
    InputTiChannel  = 0b01,
    InputTiOther    = 0b10,
    InputTrc        = 0b11
  };

  template <
          uint32_t Period
        , uint8_t CcChannel
        , TimerCaptureCompareSelection InputSelection
        , TimerDigitalFilter InputFilter
        , TimerCapturePolarity EdgeTrigger
        , TimerCapturePrescaler Prescaler >
  struct InputCaptureConfig : public TimerModeConfig<
                                      TimerMode::InputCapture, 
                                      TimerDirection::Up, 
                                      TimerCenterAlignedMode::Edge >
  {
    InputCaptureConfig()
    {
      static_assert(InputSelection != TimerCaptureCompareSelection::Output,
                    "Invalid capture input selection");
    }
    static constexpr auto period = Period;
    static constexpr auto ccChannel = CcChannel;
    static constexpr auto inputSelection = InputSelection;
    static constexpr auto inputFilter = InputFilter;
    static constexpr auto edgeTrigger = EdgeTrigger;
    static constexpr auto prescaler = Prescaler;
  };

  template< uint8_t TimerIndex >
  class GeneralPurposeTimer
  {
    public:
      static GeneralPurposeTimer& getInstance()
      {
        static GeneralPurposeTimer< TimerIndex > timer;
        ensureGeneralPurposeTimerLink = true;
        return timer;
      }

      void setPeriod(const uint32_t us)
      {
        if constexpr(TimerIndex == 2 || TimerIndex == 5)
        {
          ///@todo adjust to 32 bit values
          if(us <= 65536)
          {
            timerInstance_->PSC = CoreFrequency/1000000 - 1;  //< CNT_CLK = 1 MHz
            timeDivisionFactor_ = 1;
          }
          else if(us <= 655360)
          {
            timerInstance_->PSC = CoreFrequency/100000 - 1;  //< CNT_CLK = 100 kHz
            timeDivisionFactor_ = 10;
          }
          else if(us <= 6553600)
          {
            timerInstance_->PSC = CoreFrequency/10000 - 1;  //< CNT_CLK = 10 kHz
            timeDivisionFactor_ = 100;
          }
          else if(us <= 13107200)
          {
            timerInstance_->PSC = CoreFrequency/5000 - 1;  //< CNT_CLK = 5 kHz
            timeDivisionFactor_ = 200;
          }
          else if(us <= 26214400)
          {
            timerInstance_->PSC = CoreFrequency/2500 - 1;  //< CNT_CLK = 2.5 kHz
            timeDivisionFactor_ = 400;
          }
          else
          {
            //invalid
            timerInstance_->PSC = CoreFrequency/1000000 - 1;
            timeDivisionFactor_ = 1;
          }
        }
        else
        {
          if(us <= 65536)
          {
            timerInstance_->PSC = CoreFrequency/1000000 - 1;  //< CNT_CLK = 1 MHz
            timeDivisionFactor_ = 1;
          }
          else if(us <= 655360)
          {
            timerInstance_->PSC = CoreFrequency/100000 - 1;  //< CNT_CLK = 100 kHz
            timeDivisionFactor_ = 10;
          }
          else if(us <= 6553600)
          {
            timerInstance_->PSC = CoreFrequency/10000 - 1;  //< CNT_CLK = 10 kHz
            timeDivisionFactor_ = 100;
          }
          else if(us <= 13107200)
          {
            timerInstance_->PSC = CoreFrequency/5000 - 1;  //< CNT_CLK = 5 kHz
            timeDivisionFactor_ = 200;
          }
          else if(us <= 26214400)
          {
            timerInstance_->PSC = CoreFrequency/2500 - 1;  //< CNT_CLK = 2.5 kHz
            timeDivisionFactor_ = 400;
          }
          else
          {
            //invalid
            timerInstance_->PSC = CoreFrequency/1000000 - 1;
            timeDivisionFactor_ = 1;
          }
        }
        timerInstance_->ARR = us/timeDivisionFactor_ - 1;
        generateUpdateEvent();
      }

      template <uint8_t Channel>
      void setCaptureCompare(const uint32_t us)
      {
        if constexpr(Channel == 1)
        {
          timerInstance_->CCR1 = us/timeDivisionFactor_ - 1;
        }
        else if constexpr(Channel == 2)
        {
          timerInstance_->CCR2 = us/timeDivisionFactor_ - 1;
        }
        else if constexpr(Channel == 3)
        {
          timerInstance_->CCR3 = us/timeDivisionFactor_ - 1;
        }
        else if constexpr(Channel == 4)
        {
          timerInstance_->CCR4 = us/timeDivisionFactor_ - 1;
        }
      }

      uint32_t getCountTime() const
      {
        return (timerInstance_->CNT + 1) * timeDivisionFactor_;
      }

      template <uint8_t Channel>
      uint32_t getCaptureCompareTime() const
      {
        if constexpr(Channel == 1)
        {
          return (timerInstance_->CCR1 + 1) * timeDivisionFactor_;
        }
        else if constexpr(Channel == 2)
        {
          return (timerInstance_->CCR2 + 1) * timeDivisionFactor_;
        }
        else if constexpr(Channel == 3)
        {
          return (timerInstance_->CCR3 + 1) * timeDivisionFactor_;
        }
        else if constexpr(Channel == 4)
        {
          return (timerInstance_->CCR4 + 1) * timeDivisionFactor_;
        }
      }

      void start()
      {
        timerInstance_->CR1 |= TIM_CR1_CEN;
      }

      void stop()
      {
        timerInstance_->CR1 &= ~TIM_CR1_CEN;
      }

      template < typename ConfigT >
      void configureCaptureCompareChannel(const ConfigT& config = ConfigT())
      {
        if constexpr(config.mode == TimerMode::PwmOutput)
        {
          setPeriod(config.period);
          setCaptureCompare<config.ccChannel>(config.dutyCycle);
          selectCaptureOrCompare<config.ccChannel, TimerCaptureCompareSelection::Output>();
          setOutputPreloadEnable<config.ccChannel, true>();
          setPwmMode<config.ccChannel>();
          setPolarity<config.ccChannel, config.polarity>();
          enableCaptureCompare<config.ccChannel, config.outputEnable>();
        }
        else if constexpr(config.mode == TimerMode::InputCapture)
        {
          setPeriod(config.period);
          selectCaptureOrCompare<config.ccChannel, config.inputSelection>();
          setDigitalFilter<config.ccChannel, config.inputFilter>();
          setPrescaler<config.ccChannel, config.prescaler>();
          setEdgeTrigger<config.ccChannel, config.edgeTrigger>();
          enableCaptureCompare<config.ccChannel, true>();
        }
      }

      template <uint8_t Channel, TimerCaptureCompareSelection Selection>
      void selectCaptureOrCompare()
      {
        if constexpr(Channel == 1)
        {
          timerInstance_->CCMR1 &= ~(0x3 << TIM_CCMR1_CC1S_Pos);
          timerInstance_->CCMR1 |= static_cast<uint8_t>(Selection) << TIM_CCMR1_CC1S_Pos;
        }
        else if constexpr(Channel == 2)
        {
          timerInstance_->CCMR1 &= ~(0x3 << TIM_CCMR1_CC2S_Pos);
          timerInstance_->CCMR1 |= static_cast<uint8_t>(Selection) << TIM_CCMR1_CC2S_Pos;
        }
        else if constexpr(Channel == 3)
        {
          timerInstance_->CCMR2 &= ~(0x3 << TIM_CCMR2_CC3S_Pos);
          timerInstance_->CCMR2 |= static_cast<uint8_t>(Selection) << TIM_CCMR2_CC3S_Pos;
        }
        else if constexpr(Channel == 4)
        {
          timerInstance_->CCMR2 &= ~(0x3 << TIM_CCMR2_CC4S_Pos);
          timerInstance_->CCMR2 |= static_cast<uint8_t>(Selection) << TIM_CCMR2_CC4S_Pos;
        }
      }

      template <uint8_t Channel, TimerDigitalFilter Filter>
      void setDigitalFilter()
      {
        if constexpr(Channel == 1)
        {
          timerInstance_->CCMR1 &= ~(0xF << TIM_CCMR1_IC1F_Pos);
          timerInstance_->CCMR1 |= static_cast<uint8_t>(Filter) << TIM_CCMR1_IC1F_Pos;
        }
        else if constexpr(Channel == 2)
        {
          timerInstance_->CCMR1 &= ~(0xF << TIM_CCMR1_IC2F_Pos);
          timerInstance_->CCMR1 |= static_cast<uint8_t>(Filter) << TIM_CCMR1_IC2F_Pos;
        }
        else if constexpr(Channel == 3)
        {
          timerInstance_->CCMR2 &= ~(0xF << TIM_CCMR2_IC3F_Pos);
          timerInstance_->CCMR2 |= static_cast<uint8_t>(Filter) << TIM_CCMR2_IC3F_Pos;
        }
        else if constexpr(Channel == 4)
        {
          timerInstance_->CCMR2 &= ~(0xF << TIM_CCMR2_IC4F_Pos);
          timerInstance_->CCMR2 |= static_cast<uint8_t>(Filter) << TIM_CCMR2_IC4F_Pos;
        }
      }
      
      template <uint8_t Channel, TimerCapturePrescaler Prescaler>
      void setPrescaler()
      {
        if constexpr(Channel == 1)
        {
          timerInstance_->CCMR1 &= ~(0x3 << TIM_CCMR1_IC1PSC_Pos);
          timerInstance_->CCMR1 |= static_cast<uint8_t>(Prescaler) << TIM_CCMR1_IC1PSC_Pos;
        }
        else if constexpr(Channel == 2)
        {
          timerInstance_->CCMR1 &= ~(0x3 << TIM_CCMR1_IC2PSC_Pos);
          timerInstance_->CCMR1 |= static_cast<uint8_t>(Prescaler) << TIM_CCMR1_IC2PSC_Pos;
        }
        else if constexpr(Channel == 3)
        {
          timerInstance_->CCMR2 &= ~(0x3 << TIM_CCMR2_IC3PSC_Pos);
          timerInstance_->CCMR2 |= static_cast<uint8_t>(Prescaler) << TIM_CCMR2_IC3PSC_Pos;
        }
        else if constexpr(Channel == 4)
        {
          timerInstance_->CCMR2 &= ~(0x3 << TIM_CCMR2_IC4PSC_Pos);
          timerInstance_->CCMR2 |= static_cast<uint8_t>(Prescaler) << TIM_CCMR2_IC4PSC_Pos;
        }
      }

      template <uint8_t Channel, TimerCapturePolarity Triger>
      void setEdgeTrigger()
      {
        if constexpr(Channel == 1)
        {
          timerInstance_->CCER &= ~(0x7 << TIM_CCER_CC1P_Pos);
          timerInstance_->CCER |= static_cast<uint8_t>(Triger) << TIM_CCER_CC1P_Pos;
        }
        else if constexpr(Channel == 2)
        {
          timerInstance_->CCER &= ~(0x7 << TIM_CCER_CC2P_Pos);
          timerInstance_->CCER |= static_cast<uint8_t>(Triger) << TIM_CCER_CC2P_Pos;
        }
        else if constexpr(Channel == 3)
        {
          timerInstance_->CCER &= ~(0x7 << TIM_CCER_CC3P_Pos);
          timerInstance_->CCER |= static_cast<uint8_t>(Triger) << TIM_CCER_CC3P_Pos;
        }
        else if constexpr(Channel == 4)
        {
          timerInstance_->CCER &= ~(0x7 << TIM_CCER_CC4P_Pos);
          timerInstance_->CCER |= static_cast<uint8_t>(Triger) << TIM_CCER_CC4P_Pos;
        }
      }
      
      template <uint8_t Channel, bool Enable>
      void setOutputPreloadEnable()
      {
        if constexpr(Channel == 1)
        {
          if constexpr(Enable) timerInstance_->CCMR1 |= TIM_CCMR1_OC1PE;
          else timerInstance_->CCMR1 &= ~TIM_CCMR1_OC1PE;
        }
        else if constexpr(Channel == 2)
        {
          if constexpr(Enable) timerInstance_->CCMR1 |= TIM_CCMR1_OC2PE;
          else timerInstance_->CCMR1 &= ~TIM_CCMR1_OC2PE;
        }
        else if constexpr(Channel == 3)
        {
          if constexpr(Enable) timerInstance_->CCMR2 |= TIM_CCMR2_OC3PE;
          else timerInstance_->CCMR2 &= ~TIM_CCMR2_OC3PE;
        }
        else if constexpr(Channel == 4)
        {
          if constexpr(Enable) timerInstance_->CCMR2 |= TIM_CCMR2_OC4PE;
          else timerInstance_->CCMR2 &= ~TIM_CCMR2_OC4PE;
        }
      }

      ///@todo more basic setMode function that changes
      // the OCxM bits in the CCMRx registers
      // all modes, 0b000 to 0b111
      template <uint8_t Channel>
      void setPwmMode()
      {
        if constexpr(Channel == 1)
        {
          timerInstance_->CCMR1 &= ~(0x7 << TIM_CCMR1_OC1M_Pos);
          timerInstance_->CCMR1 |= 0b110 << TIM_CCMR1_OC1M_Pos;
        }
        else if constexpr(Channel == 2)
        {
          timerInstance_->CCMR1 &= ~(0x7 << TIM_CCMR1_OC2M_Pos);
          timerInstance_->CCMR1 |= 0b110 << TIM_CCMR1_OC2M_Pos;
        }
        else if constexpr(Channel == 3)
        {
          timerInstance_->CCMR2 &= ~(0x7 << TIM_CCMR2_OC3M_Pos);
          timerInstance_->CCMR2 |= 0b110 << TIM_CCMR2_OC3M_Pos;
        }
        else if constexpr(Channel == 4)
        {
          timerInstance_->CCMR2 &= ~(0x7 << TIM_CCMR2_OC4M_Pos);
          timerInstance_->CCMR2 |= 0b110 << TIM_CCMR2_OC4M_Pos;
        }
      }

      template <uint8_t Channel, bool Enable>
      void enableCaptureCompare()
      {
        if constexpr(Channel == 1)
        {
          if constexpr(Enable) timerInstance_->CCER |= TIM_CCER_CC1E;
          else timerInstance_->CCER &= ~TIM_CCER_CC1E;
        }
        else if constexpr(Channel == 2)
        {
          if constexpr(Enable) timerInstance_->CCER |= TIM_CCER_CC2E;
          else timerInstance_->CCER &= ~TIM_CCER_CC2E;
        }
        else if constexpr(Channel == 3)
        {
          if constexpr(Enable) timerInstance_->CCER |= TIM_CCER_CC3E;
          else timerInstance_->CCER &= ~TIM_CCER_CC3E;
        }
        else if constexpr(Channel == 4)
        {
          if constexpr(Enable) timerInstance_->CCER |= TIM_CCER_CC4E;
          else timerInstance_->CCER &= ~TIM_CCER_CC4E;
        }
      }

      template <uint8_t Channel, bool Polarity>
      void setPolarity()
      {
        if constexpr(!Polarity)
        {
          if constexpr(Channel == 1)
          {
            timerInstance_->CCER &= ~TIM_CCER_CC1P;
          }
          else if constexpr(Channel == 2)
          {
            timerInstance_->CCER &= ~TIM_CCER_CC2P;
          }
          else if constexpr(Channel == 3)
          {
            timerInstance_->CCER &= ~TIM_CCER_CC3P;
          }
          else if constexpr(Channel == 4)
          {
            timerInstance_->CCER &= ~TIM_CCER_CC4P;
          }
        }
        else
        {
          if constexpr(Channel == 1)
          {
            timerInstance_->CCER |= TIM_CCER_CC1P;
          }
          else if constexpr(Channel == 2)
          {
            timerInstance_->CCER |= TIM_CCER_CC2P;
          }
          else if constexpr(Channel == 3)
          {
            timerInstance_->CCER |= TIM_CCER_CC3P;
          }
          else if constexpr(Channel == 4)
          {
            timerInstance_->CCER |= TIM_CCER_CC4P;
          }
        }
      }

      void enableOverflowInterrupt(const std::function< void() >& callback)
      {
        overflowCallback_ = callback;
        timerInstance_->SR &= ~TIM_SR_UIF;
        timerInstance_->DIER |= TIM_DIER_UIE;

        IRQn_Type timerIRQn;
        if constexpr(TimerIndex == 1) timerIRQn = TIM1_UP_TIM10_IRQn;  
        else if constexpr(TimerIndex == 2) timerIRQn = TIM2_IRQn;
        else if constexpr(TimerIndex == 3) timerIRQn = TIM3_IRQn;
        else if constexpr(TimerIndex == 4) timerIRQn = TIM4_IRQn;
        else if constexpr(TimerIndex == 5) timerIRQn = TIM5_IRQn;
        else if constexpr(TimerIndex == 9) timerIRQn = TIM1_BRK_TIM9_IRQn;
        else if constexpr(TimerIndex == 10) timerIRQn = TIM1_UP_TIM10_IRQn;
        else if constexpr(TimerIndex == 11) timerIRQn = TIM1_TRG_COM_TIM11_IRQn;

        if(!NVIC_GetEnableIRQ(timerIRQn))
        {
          NVIC_ClearPendingIRQ(timerIRQn);
          NVIC_EnableIRQ(timerIRQn);
        }
      }

      void disableOverflowInterrupt()
      {
        overflowCallback_ = nullptr;
        timerInstance_->DIER &= ~TIM_DIER_UIE;
  
        const bool ccIrqEnabled = (timerInstance_->DIER & TIM_DIER_CC1IE) || (timerInstance_->DIER & TIM_DIER_CC2IE) ||
                                  (timerInstance_->DIER & TIM_DIER_CC3IE) || (timerInstance_->DIER & TIM_DIER_CC4IE);
        
        IRQn_Type timerIRQn;
        if constexpr(TimerIndex == 1)
        {
          ///@todo take TIM10 in considerations
          if(NVIC_GetEnableIRQ(TIM1_UP_TIM10_IRQn))
          {
            NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
            NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
          }
        }
        else
        {
          if constexpr(TimerIndex == 2) timerIRQn = TIM2_IRQn;
          else if constexpr(TimerIndex == 3) timerIRQn = TIM3_IRQn;
          else if constexpr(TimerIndex == 4) timerIRQn = TIM4_IRQn;
          else if constexpr(TimerIndex == 5) timerIRQn = TIM5_IRQn;
          else if constexpr(TimerIndex == 9) timerIRQn = TIM1_BRK_TIM9_IRQn;
          else if constexpr(TimerIndex == 10) timerIRQn = TIM1_UP_TIM10_IRQn;
          else if constexpr(TimerIndex == 11) timerIRQn = TIM1_TRG_COM_TIM11_IRQn;

          if(NVIC_GetEnableIRQ(timerIRQn) && !ccIrqEnabled)
          {
            NVIC_ClearPendingIRQ(timerIRQn);
            NVIC_DisableIRQ(timerIRQn);
          }
        }
      }

      template <uint8_t Channel>
      void enableCaptureCompareInterrupt(const std::function< void() >& callback)
      {
        IRQn_Type timerIRQn;
        if constexpr(Channel == 1)
        {
          timerInstance_->SR &= ~TIM_SR_CC1IF;
          timerInstance_->DIER |= TIM_DIER_CC1IE;
          captureCompareCallback1_ = callback;
        }
        else if constexpr(Channel == 2)
        {
          timerInstance_->SR &= ~TIM_SR_CC2IF;
          timerInstance_->DIER |= TIM_DIER_CC2IE;
          captureCompareCallback2_ = callback;
        }
        else if constexpr(Channel == 3)
        {
          timerInstance_->SR &= ~TIM_SR_CC3IF;
          timerInstance_->DIER |= TIM_DIER_CC3IE;
          captureCompareCallback3_ = callback;
        }
        else if constexpr(Channel == 4)
        {
          timerInstance_->SR &= ~TIM_SR_CC4IF;
          timerInstance_->DIER |= TIM_DIER_CC4IE;
          captureCompareCallback4_ = callback;
        }
        if constexpr(TimerIndex == 1) timerIRQn = TIM1_CC_IRQn;
        else if constexpr(TimerIndex == 2) timerIRQn = TIM2_IRQn;
        else if constexpr(TimerIndex == 3) timerIRQn = TIM3_IRQn;
        else if constexpr(TimerIndex == 4) timerIRQn = TIM4_IRQn;
        else if constexpr(TimerIndex == 5) timerIRQn = TIM5_IRQn;
        else if constexpr(TimerIndex == 9) timerIRQn = TIM1_BRK_TIM9_IRQn;
        else if constexpr(TimerIndex == 10) timerIRQn = TIM1_UP_TIM10_IRQn;
        else if constexpr(TimerIndex == 11) timerIRQn = TIM1_TRG_COM_TIM11_IRQn;
        
        if(!NVIC_GetEnableIRQ(timerIRQn))
        {
          NVIC_ClearPendingIRQ(timerIRQn);
          NVIC_EnableIRQ(timerIRQn);
        }
      }
      // template<uint8_t CcIndex>
      // void setCaptureCompare(const uint32_t us)
      // {
      //   // ovaa funkcija ja naprajmve so template 
      //   // (namesto setCaptureCompare(const uint8_t ccIndex, const uint32_t us))
      //   // zosto nikoj ne bi imal potreba capture compare pri runtime da gi
      //   // pristapuvat, so nekoja si runtime promenliva (nikoj nemat da piset kod)
      //   // for(int ccIndex = 0; ccIndex < 5; ccIndex++) setCaptureCompare(ccIndex, 10);
      //   // primer. Zatoa so templates, ogranicuvame CCindex da e poznato pri compile time,
      //   // znaci povik so setCaptureCompare<0>(30);  // CC0 e namesten da naprajt compare na 30 us
      //   // Sepak ovaa funkcija trebat da se rafinirat, ne e vo wed vaka, zoco samo za compare mode
      //   // mojc vremeto da go namestis.
      //   // znaci napraj enum class gore ete napicaw gowe da vidic enum class
      // }

      template <uint8_t Channel>
      void disableCaptureCompareInterrupt(const std::function< void() >& callback)
      {
        (void)callback;
      }

    private:
      GeneralPurposeTimer()
      {
        static_assert((TimerIndex >= 1 && TimerIndex <= 5) || (TimerIndex >= 9 && TimerIndex <= 11)
                  , "Invalid Timer instance requested");

        if constexpr(TimerIndex == 1)
        {
          RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
        }
        else if constexpr(TimerIndex == 2)
        {
          RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
        }
        else if constexpr(TimerIndex == 3)
        {
          RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
        }
        else if constexpr(TimerIndex == 4)
        {
          RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
        }
        else if constexpr(TimerIndex == 5)
        {
          RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
        }
        else if constexpr(TimerIndex == 9)
        {
          RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
        }
        else if constexpr(TimerIndex == 10)
        {
          RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
        }
        else if constexpr(TimerIndex == 11)
        {
          RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
        }
      }
      GeneralPurposeTimer(const GeneralPurposeTimer&) = delete;
      GeneralPurposeTimer(GeneralPurposeTimer&&) = delete;
      void operator=(const GeneralPurposeTimer&) = delete;
      void operator=(GeneralPurposeTimer&&) = delete;

      void generateUpdateEvent()
      {
        timerInstance_->EGR |= TIM_EGR_UG;
        timerInstance_->SR &= ~TIM_SR_UIF;
      }

      constexpr static TIM_TypeDef* getTimerInstance(const uint8_t timerIndex)
      {
        switch(timerIndex)
        {
          case 1:   return TIM1;
          break;
          case 2:   return TIM2;
          break;
          case 3:   return TIM3;
          break;
          case 4:   return TIM4;
          break;
          case 5:   return TIM5;
          break;
          case 9:   return TIM9;
          break;
          case 10:  return TIM10;
          break;
          case 11:  return TIM11;
          break;
          default:  return nullptr;
          break;
        }
      }
      TIM_TypeDef * const timerInstance_ = getTimerInstance(TimerIndex);
      uint32_t timeDivisionFactor_;
    // Interrupt callbacks
    public:
      inline static std::function< void() > overflowCallback_;
      inline static std::function< void() > captureCompareCallback1_;
      inline static std::function< void() > captureCompareCallback2_;
      inline static std::function< void() > captureCompareCallback3_;
      inline static std::function< void() > captureCompareCallback4_;
  };
}

#endif //STM32_GENERAL_PURPOSE_TIMER_HPP 