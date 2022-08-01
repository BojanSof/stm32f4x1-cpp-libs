#include "GeneralPurposeTimer.hpp"

namespace Stm32
{
  bool ensureGeneralPurposeTimerLink = false;
}

extern "C"
{
  void TIM1_CC_IRQHandler()
  {
    using namespace Stm32;
    if(TIM1->SR & TIM_SR_CC1IF)
    {
      if(GeneralPurposeTimer<1>::captureCompareCallback1_)
      {
        GeneralPurposeTimer<1>::captureCompareCallback1_();
      }
      TIM1->SR &= ~TIM_SR_CC1IF;
    }
    else if(TIM1->SR & TIM_SR_CC1IF)
    {
      if(GeneralPurposeTimer<1>::captureCompareCallback1_)
      {
        GeneralPurposeTimer<1>::captureCompareCallback1_();
      }
      TIM1->SR &= ~TIM_SR_CC1IF;
    }
    else if(TIM1->SR & TIM_SR_CC2IF)
    {
      if(GeneralPurposeTimer<1>::captureCompareCallback2_)
      {
        GeneralPurposeTimer<1>::captureCompareCallback2_();
      }
      TIM1->SR &= ~TIM_SR_CC2IF;
    }
    else if(TIM1->SR & TIM_SR_CC3IF)
    {
      if(GeneralPurposeTimer<1>::captureCompareCallback3_)
      {
        GeneralPurposeTimer<1>::captureCompareCallback3_();
      }
      TIM1->SR &= ~TIM_SR_CC3IF;
    }
    else if(TIM1->SR & TIM_SR_CC4IF)
    {
      if(GeneralPurposeTimer<1>::captureCompareCallback4_)
      {
        GeneralPurposeTimer<1>::captureCompareCallback4_();
      }
      TIM1->SR &= ~TIM_SR_CC4IF;
    }
  }

  void TIM1_UP_TIM10_IRQHandler()
  {
    using namespace Stm32;
    if(TIM1->SR & TIM_SR_UIF)
    {
      if(GeneralPurposeTimer<1>::overflowCallback_)
      {
        GeneralPurposeTimer<1>::overflowCallback_();
      }
      TIM1->SR &= ~TIM_SR_UIF; //< clear the flag
    }
    else if(TIM10->SR & TIM_SR_UIF)
    {
      if(GeneralPurposeTimer<10>::overflowCallback_)
      {
        GeneralPurposeTimer<10>::overflowCallback_();
      }
      TIM10->SR &= ~TIM_SR_UIF; //< clear the flag
    }
    else if(TIM10->SR & TIM_SR_CC1IF)
    {
      if(GeneralPurposeTimer<10>::captureCompareCallback1_)
      {
        GeneralPurposeTimer<10>::captureCompareCallback1_();
      }
      TIM10->SR &= ~TIM_SR_CC1IF;
    }
    else if(TIM10->SR & TIM_SR_CC2IF)
    {
      if(GeneralPurposeTimer<10>::captureCompareCallback2_)
      {
        GeneralPurposeTimer<10>::captureCompareCallback2_();
      }
      TIM10->SR &= ~TIM_SR_CC2IF;
    }
    else if(TIM10->SR & TIM_SR_CC3IF)
    {
      if(GeneralPurposeTimer<10>::captureCompareCallback3_)
      {
        GeneralPurposeTimer<10>::captureCompareCallback3_();
      }
      TIM10->SR &= ~TIM_SR_CC3IF;
    }
    else if(TIM10->SR & TIM_SR_CC4IF)
    {
      if(GeneralPurposeTimer<10>::captureCompareCallback4_)
      {
        GeneralPurposeTimer<10>::captureCompareCallback4_();
      }
      TIM10->SR &= ~TIM_SR_CC4IF;
    }
  }

  void TIM2_IRQHandler()
  {
    using namespace Stm32;
    if(TIM2->SR & TIM_SR_UIF)
    {
      if(GeneralPurposeTimer<2>::overflowCallback_)
      {
        GeneralPurposeTimer<2>::overflowCallback_();
      }
      TIM2->SR &= ~TIM_SR_UIF;
    }
    else if(TIM2->SR & TIM_SR_CC1IF)
    {
      if(GeneralPurposeTimer<2>::captureCompareCallback1_)
      {
        GeneralPurposeTimer<2>::captureCompareCallback1_();
      }
      TIM2->SR &= ~TIM_SR_CC1IF;
    }
    else if(TIM2->SR & TIM_SR_CC2IF)
    {
      if(GeneralPurposeTimer<2>::captureCompareCallback2_)
      {
        GeneralPurposeTimer<2>::captureCompareCallback2_();
      }
      TIM2->SR &= ~TIM_SR_CC2IF;
    }
    else if(TIM2->SR & TIM_SR_CC3IF)
    {
      if(GeneralPurposeTimer<2>::captureCompareCallback3_)
      {
        GeneralPurposeTimer<2>::captureCompareCallback3_();
      }
      TIM2->SR &= ~TIM_SR_CC3IF;
    }
    else if(TIM2->SR & TIM_SR_CC4IF)
    {
      if(GeneralPurposeTimer<2>::captureCompareCallback4_)
      {
        GeneralPurposeTimer<2>::captureCompareCallback4_();
      }
      TIM2->SR &= ~TIM_SR_CC4IF;
    }
  }

  void TIM3_IRQHandler()
  {
    using namespace Stm32;
    if(TIM3->SR & TIM_SR_UIF)
    {
      if(GeneralPurposeTimer<3>::overflowCallback_)
      {
        GeneralPurposeTimer<3>::overflowCallback_();
      }
      TIM3->SR &= ~TIM_SR_UIF;
    }
    else if(TIM3->SR & TIM_SR_CC1IF)
    {
      if(GeneralPurposeTimer<3>::captureCompareCallback1_)
      {
        GeneralPurposeTimer<3>::captureCompareCallback1_();
      }
      TIM3->SR &= ~TIM_SR_CC1IF;
    }
    else if(TIM3->SR & TIM_SR_CC2IF)
    {
      if(GeneralPurposeTimer<3>::captureCompareCallback2_)
      {
        GeneralPurposeTimer<3>::captureCompareCallback2_();
      }
      TIM3->SR &= ~TIM_SR_CC2IF;
    }
    else if(TIM3->SR & TIM_SR_CC3IF)
    {
      if(GeneralPurposeTimer<3>::captureCompareCallback3_)
      {
        GeneralPurposeTimer<3>::captureCompareCallback3_();
      }
      TIM3->SR &= ~TIM_SR_CC3IF;
    }
    else if(TIM3->SR & TIM_SR_CC4IF)
    {
      if(GeneralPurposeTimer<3>::captureCompareCallback4_)
      {
        GeneralPurposeTimer<3>::captureCompareCallback4_();
      }
      TIM3->SR &= ~TIM_SR_CC4IF;
    }
  }

  void TIM4_IRQHandler()
  {
    using namespace Stm32;
    if(TIM4->SR & TIM_SR_UIF)
    {
      if(GeneralPurposeTimer<4>::overflowCallback_)
      {
        GeneralPurposeTimer<4>::overflowCallback_();
      }
      TIM4->SR &= ~TIM_SR_UIF;
    }
    else if(TIM4->SR & TIM_SR_CC1IF)
    {
      if(GeneralPurposeTimer<4>::captureCompareCallback1_)
      {
        GeneralPurposeTimer<4>::captureCompareCallback1_();
      }
      TIM4->SR &= ~TIM_SR_CC1IF;
    }
    else if(TIM4->SR & TIM_SR_CC2IF)
    {
      if(GeneralPurposeTimer<4>::captureCompareCallback2_)
      {
        GeneralPurposeTimer<4>::captureCompareCallback2_();
      }
      TIM4->SR &= ~TIM_SR_CC2IF;
    }
    else if(TIM4->SR & TIM_SR_CC3IF)
    {
      if(GeneralPurposeTimer<4>::captureCompareCallback3_)
      {
        GeneralPurposeTimer<4>::captureCompareCallback3_();
      }
      TIM4->SR &= ~TIM_SR_CC3IF;
    }
    else if(TIM4->SR & TIM_SR_CC4IF)
    {
      if(GeneralPurposeTimer<4>::captureCompareCallback4_)
      {
        GeneralPurposeTimer<4>::captureCompareCallback4_();
      }
      TIM4->SR &= ~TIM_SR_CC4IF;
    }
  }

  void TIM5_IRQHandler()
  {
    using namespace Stm32;
    if(TIM5->SR & TIM_SR_UIF)
    {
      if(GeneralPurposeTimer<5>::overflowCallback_)
      {
        GeneralPurposeTimer<5>::overflowCallback_();
      }
      TIM5->SR &= ~TIM_SR_UIF;
    }
    else if(TIM5->SR & TIM_SR_CC1IF)
    {
      if(GeneralPurposeTimer<5>::captureCompareCallback1_)
      {
        GeneralPurposeTimer<5>::captureCompareCallback1_();
      }
      TIM5->SR &= ~TIM_SR_CC1IF;
    }
    else if(TIM5->SR & TIM_SR_CC2IF)
    {
      if(GeneralPurposeTimer<5>::captureCompareCallback2_)
      {
        GeneralPurposeTimer<5>::captureCompareCallback2_();
      }
      TIM5->SR &= ~TIM_SR_CC2IF;
    }
    else if(TIM5->SR & TIM_SR_CC3IF)
    {
      if(GeneralPurposeTimer<5>::captureCompareCallback3_)
      {
        GeneralPurposeTimer<5>::captureCompareCallback3_();
      }
      TIM5->SR &= ~TIM_SR_CC3IF;
    }
    else if(TIM5->SR & TIM_SR_CC4IF)
    {
      if(GeneralPurposeTimer<5>::captureCompareCallback4_)
      {
        GeneralPurposeTimer<5>::captureCompareCallback4_();
      }
      TIM5->SR &= ~TIM_SR_CC4IF;
    }
  }

  void TIM1_BRK_TIM9_IRQHandler()
  {
    using namespace Stm32;
    if(TIM9->SR & TIM_SR_UIF)
    {
      if(GeneralPurposeTimer<9>::overflowCallback_)
      {
        GeneralPurposeTimer<9>::overflowCallback_();
      }
      TIM9->SR &= ~TIM_SR_UIF;
    }
    else if(TIM9->SR & TIM_SR_CC1IF)
    {
      if(GeneralPurposeTimer<9>::captureCompareCallback1_)
      {
        GeneralPurposeTimer<9>::captureCompareCallback1_();
      }
      TIM9->SR &= ~TIM_SR_CC1IF;
    }
    else if(TIM9->SR & TIM_SR_CC2IF)
    {
      if(GeneralPurposeTimer<9>::captureCompareCallback2_)
      {
        GeneralPurposeTimer<9>::captureCompareCallback2_();
      }
      TIM9->SR &= ~TIM_SR_CC2IF;
    }
    else if(TIM9->SR & TIM_SR_CC3IF)
    {
      if(GeneralPurposeTimer<9>::captureCompareCallback3_)
      {
        GeneralPurposeTimer<9>::captureCompareCallback3_();
      }
      TIM9->SR &= ~TIM_SR_CC3IF;
    }
    else if(TIM9->SR & TIM_SR_CC4IF)
    {
      if(GeneralPurposeTimer<9>::captureCompareCallback4_)
      {
        GeneralPurposeTimer<9>::captureCompareCallback4_();
      }
      TIM9->SR &= ~TIM_SR_CC4IF;
    }
  }

  void TIM1_TRG_COM_TIM11_IRQHandler()
  {
    using namespace Stm32;
    if(TIM11->SR & TIM_SR_UIF)
    {
      if(GeneralPurposeTimer<11>::overflowCallback_)
      {
        GeneralPurposeTimer<11>::overflowCallback_();
      }
      TIM11->SR &= ~TIM_SR_UIF;
    }
    else if(TIM11->SR & TIM_SR_CC1IF)
    {
      if(GeneralPurposeTimer<11>::captureCompareCallback1_)
      {
        GeneralPurposeTimer<11>::captureCompareCallback1_();
      }
      TIM11->SR &= ~TIM_SR_CC1IF;
    }
    else if(TIM11->SR & TIM_SR_CC2IF)
    {
      if(GeneralPurposeTimer<11>::captureCompareCallback2_)
      {
        GeneralPurposeTimer<11>::captureCompareCallback2_();
      }
      TIM11->SR &= ~TIM_SR_CC2IF;
    }
    else if(TIM11->SR & TIM_SR_CC3IF)
    {
      if(GeneralPurposeTimer<11>::captureCompareCallback3_)
      {
        GeneralPurposeTimer<11>::captureCompareCallback3_();
      }
      TIM11->SR &= ~TIM_SR_CC3IF;
    }
    else if(TIM11->SR & TIM_SR_CC4IF)
    {
      if(GeneralPurposeTimer<11>::captureCompareCallback4_)
      {
        GeneralPurposeTimer<11>::captureCompareCallback4_();
      }
      TIM11->SR &= ~TIM_SR_CC4IF;
    }
  }
}