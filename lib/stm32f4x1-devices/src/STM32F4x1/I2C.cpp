#include "STM32F4x1/I2C.hpp"

namespace Stm32
{
  bool ensureI2CLink = false;
}

extern "C"
{
  void I2C1_EV_IRQHandler()
  {
    using namespace Stm32;
    if(I2C<1>::transferCallback_)
    {
      I2C<1>::transferCallback_();
    }
  }

  void I2C1_ER_IRQHandler()
  {
    using namespace Stm32;
    if(I2C<1>::errorCallback_)
    {
      I2C<1>::errorCallback_();
    }
  }

  void I2C2_EV_IRQHandler()
  {
    using namespace Stm32;
    if(I2C<2>::transferCallback_)
    {
      I2C<2>::transferCallback_();
    }
  }

  void I2C2_ER_IRQHandler()
  {
    using namespace Stm32;
    if(I2C<2>::errorCallback_)
    {
      I2C<2>::errorCallback_();
    }   
  }

  void I2C3_EV_IRQHandler()
  {
    using namespace Stm32;
    if(I2C<3>::transferCallback_)
    {
      I2C<3>::transferCallback_();
    }
  }

  void I2C3_ER_IRQHandler()
  {
    using namespace Stm32;
    if(I2C<3>::errorCallback_)
    {
      I2C<3>::errorCallback_();
    }    
  }
}