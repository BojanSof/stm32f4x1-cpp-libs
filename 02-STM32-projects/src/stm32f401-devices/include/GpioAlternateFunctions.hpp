#ifndef STM32_GPIO_ALTERNATE_FUNCTIONS_HPP
#define STM32_GPIO_ALTERNATE_FUNCTIONS_HPP

#include "GpioTypes.hpp"

#include <cstdint>

namespace Stm32
{
  /**
   * Enumeration which holds all the possible alternate
   * functions and can be converted to two hex digits
   * number 0xXYZ, where X is the number of the alternate function
   * (0 to F correspond to AF0 to AF15), and YZ is the unique
   * digit for every alternate function.
   * 
   */
  enum class GpioAlternateFunctionId : uint16_t
  {
    Default                 =   0x000,
    // AF00 (SYS_AF)
    MCO_1                   =   0x001,
    JTMS_SWDIO              =   0x002,
    JTCK_SWCLK              =   0x003,
    JTDI                    =   0x004,
    JTDO_SWO                =   0x005,
    JTRST                   =   0x006,
    RTC_REFN                =   0x007,
    MCO_2                   =   0x008,
    TRACECLK                =   0x009,
    TRACED0                 =   0x00A,
    TRACED1                 =   0x00B,
    TRACED2                 =   0x00C,
    TRACED3                 =   0x00D,
    // AF1 (TIM1/TIM2)
    TIM1_BKIN               =   0x100,
    TIM1_ETR                =   0x101,
    TIM1_CH1                =   0x102,
    TIM1_CH2                =   0x103,
    TIM1_CH3                =   0x104,
    TIM1_CH4                =   0x105,
    TIM1_CH1N               =   0x106,
    TIM1_CH2N               =   0x107,
    TIM1_CH3N               =   0x108,
    TIM2_CH1                =   0x109,
    TIM2_CH2                =   0x10A,
    TIM2_CH3                =   0x10B,
    TIM2_CH4                =   0x10C,
    // AF2 (TIM3/TIM4/TIM5)
    TIM3_ETR                =   0x200,
    TIM3_CH1                =   0x201,
    TIM3_CH2                =   0x202,
    TIM3_CH3                =   0x203,
    TIM3_CH4                =   0x204,
    TIM4_ETR                =   0x205,
    TIM4_CH1                =   0x206,
    TIM4_CH2                =   0x207,
    TIM4_CH3                =   0x208,
    TIM4_CH4                =   0x209,
    TIM5_CH1                =   0x20A,
    TIM5_CH2                =   0x20B,
    TIM5_CH3                =   0x20C,
    TIM5_CH4                =   0x20D,
    // AF3 (TIM9/TIM10/TIM11)
    TIM9_CH1                =   0x300,
    TIM9_CH2                =   0x301,
    TIM10_CH1               =   0x302,
    TIM11_CH1               =   0x303,
    // AF4 (I2C1/I2C2/I2C3)
    I2C1_SCL                =   0x400,
    I2C1_SDA                =   0x401,
    I2C1_SMBA               =   0x402,
    I2C2_SCL                =   0x403,
    I2C2_SDA_AF4            =   0x404,
    I2C2_SMBA               =   0x405,
    I2C3_SCL                =   0x406,
    I2C3_SDA_AF4            =   0x407,
    I2C3_SMBA               =   0x408,
    // AF5 (SPI1/SPI2/I2S2/SPI3/I2S3/SPI4)
    SPI1_SCK                =   0x500,
    SPI1_MISO               =   0x501,
    SPI1_MOSI               =   0x502,
    SPI1_NSS                =   0x503,
    SPI2_SCK                =   0x504,
    SPI2_MISO               =   0x505,
    SPI2_MOSI               =   0x506,
    SPI2_NSS                =   0x507,
    SPI3_MOSI_AF5           =   0x508,
    SPI4_SCK                =   0x509,
    SPI4_MISO               =   0x50A,
    SPI4_MOSI               =   0x50B,
    SPI4_NSS                =   0x50C,
    I2S2_WS                 =   0x50D,
    I2S2_CK                 =   0x50E,
    I2S2_SD                 =   0x50F,
    I2S2_MCK                =   0x510,
    I2S2_CKIN               =   0x511,
    I2S3_SD_AF5             =   0x512,
    I2S3ext_SD_AF5          =   0x513,
    // AF06 (SPI2/I2S2/SPI3/I2S3)
    SPI3_NSS                =   0x600,
    SPI3_SCK                =   0x601,
    SPI3_MISO               =   0x602,
    SPI3_MOSI_AF6           =   0x603,
    I2S2ext_SD              =   0x604,
    I2S3_WS                 =   0x605,
    I2S3_CK                 =   0x606,
    I2S3_SD_AF6             =   0x607,
    I2S3_MCK                =   0x608,
    // AF07 (SPI3/I2S3/USART1/USART2)
    USART1_TX               =   0x700,
    USART1_RX               =   0x701,
    USART1_CTS              =   0x702,
    USART1_RTS              =   0x703,
    USART1_CK               =   0x704,
    USART2_TX               =   0x705,
    USART2_RX               =   0x706,
    USART2_CTS              =   0x707,
    USART2_RTS              =   0x708,
    USART2_CK               =   0x709,
    I2S3ext_SD_AF7          =   0x70A,
    // AF08 (USART6)
    USART6_TX               =   0x800,
    USART6_RX               =   0x801,
    USART6_CK               =   0x802,
    // AF09 (I2C2/I2C3)
    I2C2_SDA_AF9            =   0x900,
    I2C3_SDA_AF9            =   0x901,
    // AF10 (OTG1_FS)
    OTG_FS_SOF              =   0xA00,
    OTG_FS_VBUS             =   0xA01,
    OTG_FS_ID               =   0xA02,
    OTG_FS_DM               =   0xA03,
    OTG_FS_DP               =   0xA04,
    // AF11 (empty)
    // AF12 (SDIO)
    SDIO_CMD                =   0xC00,
    SDIO_CK                 =   0xC01,
    SDIO_D0                 =   0xC02,
    SDIO_D1                 =   0xC03,
    SDIO_D2                 =   0xC04,
    SDIO_D3                 =   0xC05,
    SDIO_D4                 =   0xC06,
    SDIO_D5                 =   0xC07,
    SDIO_D6                 =   0xC08,
    SDIO_D7                 =   0xC09,
    // AF13 (empty)
    // AF14 (empty)
    // AF15
    EVENTOUT                =   0xF00
  };

  template <Port port, uint8_t pin>
  struct GpioAlternateFunction
  {
    enum Type : uint16_t
    {
      Default = static_cast<uint16_t>(GpioAlternateFunctionId::Default)
    };

    static constexpr Type types[] = {Default};  //< to check if given AF exists for this pin
  };

  /**
   * @brief Check if the specified alternate function exists
   * for a given pin
   * 
   * @tparam port The port where the pin belongs
   * @tparam pin The number of the pin on the port
   * @param af The alternate function requested for check
   * @return true The alternate function for the specified pin exists
   * @return false The alternate function for the specified pin does not exist
   */
  template <Port port, uint8_t pin>
  constexpr bool gpioCheckAlternateFunction(const GpioAlternateFunctionId& af)
  {
    using GpioAfType = GpioAlternateFunction<port, pin>;
    for(const auto &gpioAf : GpioAfType::types_)
    {
      if(gpioAf == static_cast<typename GpioAfType::Type>(af)) return true;
    }
    return false;
  }

  // Specialize the alternate functions template class for each pin
  template<>
  struct GpioAlternateFunction<Port::A, 0>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM2_CH1      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM2_CH1),
      TIM5_CH1      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM5_CH1),
      USART2_CTS    =   static_cast<uint16_t>(GpioAlternateFunctionId::USART2_CTS),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM2_CH1, TIM5_CH1, USART2_CTS, EVENT_OUT};
  };
  ///@todo Specialize for other pins
}

#endif //STM32_GPIO_ALTERNATE_FUNCTIONS_HPP