#include "SPI.hpp"

namespace Stm32
{
  bool ensureSpiLink = false;
}

extern "C"
{
  void SPI1_Handler()
  {
    using namespace Stm32;
    if(SPI<1>::transferCallback_)
    {
      SPI<1>::transferCallback_();
    }
  }

  void SPI2_Handler()
  {
    using namespace Stm32;
    if(SPI<2>::transferCallback_)
    {
      SPI<2>::transferCallback_();
    }
  }

  void SPI3_Handler()
  {
    using namespace Stm32;
    if(SPI<3>::transferCallback_)
    {
      SPI<3>::transferCallback_();
    }
  }

  void SPI4_Handler()
  {
    using namespace Stm32;
    if(SPI<4>::transferCallback_)
    {
      SPI<4>::transferCallback_();
    }
  }
}