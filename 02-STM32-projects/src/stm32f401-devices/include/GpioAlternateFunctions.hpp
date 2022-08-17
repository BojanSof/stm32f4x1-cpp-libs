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
    TIM2_ETR                =   0x109,
    TIM2_CH1                =   0x10A,
    TIM2_CH2                =   0x10B,
    TIM2_CH3                =   0x10C,
    TIM2_CH4                =   0x10D,
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
      TIM2_ETR      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM2_ETR),
      TIM5_CH1      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM5_CH1),
      USART2_CTS    =   static_cast<uint16_t>(GpioAlternateFunctionId::USART2_CTS),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM2_CH1, TIM2_ETR, TIM5_CH1, USART2_CTS, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 1>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM2_CH2      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM2_CH2),
      TIM5_CH2      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM5_CH2),
      USART2_RTS    =   static_cast<uint16_t>(GpioAlternateFunctionId::USART2_RTS),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM2_CH2, TIM5_CH2, USART2_RTS, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 2>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM2_CH3      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM2_CH3),
      TIM5_CH3      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM5_CH3),
      TIM9_CH1      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM9_CH1),
      USART2_TX     =   static_cast<uint16_t>(GpioAlternateFunctionId::USART2_TX),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM2_CH3, TIM5_CH3, TIM9_CH1, USART2_TX, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 3>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM2_CH4      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM2_CH4),
      TIM5_CH4      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM5_CH4),
      TIM9_CH2      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM9_CH2),
      USART2_RX     =   static_cast<uint16_t>(GpioAlternateFunctionId::USART2_RX),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM2_CH4, TIM5_CH4, TIM9_CH2, USART2_RX, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 4>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      SPI1_NSS      =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI1_NSS),
      SPI3_NSS      =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI3_NSS),
      I2S3_WS       =   static_cast<uint16_t>(GpioAlternateFunctionId::I2S3_WS),
      USART2_CK     =   static_cast<uint16_t>(GpioAlternateFunctionId::USART2_CK),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, SPI1_NSS, SPI3_NSS, I2S3_WS, USART2_CK, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 5>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM2_CH1      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM2_CH1),
      TIM2_ETR      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM2_ETR),
      SPI1_SCK      =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI1_SCK),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM2_CH1, TIM2_ETR, SPI1_SCK, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 6>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM1_BKIN     =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM1_BKIN),
      TIM3_CH1      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM3_CH1),
      SPI1_MISO     =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI1_MISO),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM1_BKIN, TIM3_CH1, SPI1_MISO, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 7>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM1_CH1N     =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM1_CH1N),
      TIM3_CH2      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM3_CH2),
      SPI1_MOSI     =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI1_MOSI),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM1_CH1N, TIM3_CH2, SPI1_MOSI, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 8>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      MCO_1         =   static_cast<uint16_t>(GpioAlternateFunctionId::MCO_1),
      TIM1_CH1      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM1_CH1),
      I2C3_SCL      =   static_cast<uint16_t>(GpioAlternateFunctionId::I2C3_SCL),
      USART1_CK     =   static_cast<uint16_t>(GpioAlternateFunctionId::USART1_CK),
      OTG_FS_SOF    =   static_cast<uint16_t>(GpioAlternateFunctionId::OTG_FS_SOF),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, MCO_1, TIM1_CH1, I2C3_SCL, USART1_CK, OTG_FS_SOF, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 9>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM1_CH2      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM1_CH2),
      I2C3_SMBA     =   static_cast<uint16_t>(GpioAlternateFunctionId::I2C3_SMBA),
      USART1_TX     =   static_cast<uint16_t>(GpioAlternateFunctionId::USART1_TX),
      OTG_FS_VBUS   =   static_cast<uint16_t>(GpioAlternateFunctionId::OTG_FS_VBUS),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM1_CH2, I2C3_SMBA, USART1_TX, OTG_FS_VBUS, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 10>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM1_CH3      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM1_CH3),
      USART1_RX     =   static_cast<uint16_t>(GpioAlternateFunctionId::USART1_RX),
      OTG_FS_ID   =   static_cast<uint16_t>(GpioAlternateFunctionId::OTG_FS_ID),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM1_CH3, USART1_RX, OTG_FS_ID, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 11>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM1_CH4      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM1_CH4),
      USART1_CTS    =   static_cast<uint16_t>(GpioAlternateFunctionId::USART1_CTS),
      USART6_TX     =   static_cast<uint16_t>(GpioAlternateFunctionId::USART6_TX),
      OTG_FS_DM     =   static_cast<uint16_t>(GpioAlternateFunctionId::OTG_FS_DM),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM1_CH4, USART1_CTS, USART6_TX, OTG_FS_DM, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 12>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM1_ETR      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM1_ETR),
      USART1_RTS    =   static_cast<uint16_t>(GpioAlternateFunctionId::USART1_RTS),
      USART6_RX     =   static_cast<uint16_t>(GpioAlternateFunctionId::USART6_RX),
      OTG_FS_DP     =   static_cast<uint16_t>(GpioAlternateFunctionId::OTG_FS_DP),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM1_ETR, USART1_RTS, USART6_RX, OTG_FS_DP, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 13>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      JTMS_SWDIO    =   static_cast<uint16_t>(GpioAlternateFunctionId::JTMS_SWDIO),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, JTMS_SWDIO, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 14>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      JTCK_SWCLK    =   static_cast<uint16_t>(GpioAlternateFunctionId::JTCK_SWCLK),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, JTCK_SWCLK, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::A, 15>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      JTDI          =   static_cast<uint16_t>(GpioAlternateFunctionId::JTDI),
      TIM2_CH1      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM2_CH1),
      TIM2_ETR      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM2_ETR),
      SPI1_NSS      =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI1_NSS),
      SPI3_NSS      =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI3_NSS),
      I2S3_WS       =   static_cast<uint16_t>(GpioAlternateFunctionId::I2S3_WS),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, JTDI, TIM2_CH1, TIM2_ETR, SPI1_NSS, SPI3_NSS, I2S3_WS, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::B, 0>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM1_CH2N     =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM1_CH2N),
      TIM3_CH3      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM3_CH3),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM1_CH2N, TIM3_CH3, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::B, 1>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM1_CH3N     =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM1_CH3N),
      TIM3_CH4      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM3_CH4),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM1_CH3N, TIM3_CH4, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::B, 2>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::B, 3>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      JTDO_SWO      =   static_cast<uint16_t>(GpioAlternateFunctionId::JTDO_SWO),
      TIM2_CH2      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM2_CH2),
      SPI1_SCK      =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI1_SCK),
      SPI3_SCK      =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI3_SCK),
      I2S3_CK       =   static_cast<uint16_t>(GpioAlternateFunctionId::I2S3_CK),
      I2C2_SDA_AF9  =   static_cast<uint16_t>(GpioAlternateFunctionId::I2C2_SDA_AF9),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, JTDO_SWO, TIM2_CH2, SPI1_SCK, SPI3_SCK, I2S3_CK, I2C2_SDA_AF9, EVENT_OUT};
  };
  template<>
  struct GpioAlternateFunction<Port::B, 4>
  {
    enum Type : uint16_t
    {
      Default        =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      JTRST          =   static_cast<uint16_t>(GpioAlternateFunctionId::JTRST),
      TIM3_CH1       =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM3_CH1),
      SPI1_MISO      =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI1_MISO),
      SPI3_MISO      =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI3_MISO),
      I2S3ext_SD_AF7 =   static_cast<uint16_t>(GpioAlternateFunctionId::I2S3ext_SD_AF7),
      I2C3_SDA_AF9   =   static_cast<uint16_t>(GpioAlternateFunctionId::I2C3_SDA_AF9),
      EVENT_OUT      =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, JTRST, TIM3_CH1, SPI1_MISO, SPI3_MISO, I2S3ext_SD_AF7, I2C3_SDA_AF9, EVENT_OUT};
  };
  
  template<>
  struct GpioAlternateFunction<Port::B, 5>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM3_CH2      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM3_CH2),
      I2C1_SMBA     =   static_cast<uint16_t>(GpioAlternateFunctionId::I2C1_SMBA),
      SPI1_MOSI     =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI1_MOSI),
      SPI3_MOSI_AF6 =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI3_MOSI_AF6),
      I2S3_SD_AF6   =   static_cast<uint16_t>(GpioAlternateFunctionId::I2S3_SD_AF6),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM3_CH2, I2C1_SMBA, SPI1_MOSI, SPI3_MOSI_AF6, I2S3_SD_AF6, EVENT_OUT};
  };

  template<>
  struct GpioAlternateFunction<Port::B, 6>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM4_CH1      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM4_CH1),
      I2C1_SCL      =   static_cast<uint16_t>(GpioAlternateFunctionId::I2C1_SCL),
      USART1_TX     =   static_cast<uint16_t>(GpioAlternateFunctionId::USART1_TX),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM4_CH1, I2C1_SCL, USART1_TX, EVENT_OUT};
  };

  template<>
  struct GpioAlternateFunction<Port::B, 7>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM4_CH2      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM4_CH2),
      I2C1_SDA      =   static_cast<uint16_t>(GpioAlternateFunctionId::I2C1_SDA),
      USART1_RX     =   static_cast<uint16_t>(GpioAlternateFunctionId::USART1_RX),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM4_CH2, I2C1_SDA, USART1_RX, EVENT_OUT};
  };

  template<>
  struct GpioAlternateFunction<Port::B, 8>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM4_CH3      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM4_CH3),
      TIM10_CH1     =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM10_CH1),
      I2C1_SCL      =   static_cast<uint16_t>(GpioAlternateFunctionId::I2C1_SCL),
      SDIO_D4       =   static_cast<uint16_t>(GpioAlternateFunctionId::SDIO_D4),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM4_CH3, TIM10_CH1, I2C1_SCL, SDIO_D4, EVENT_OUT};
  };

  template<>
  struct GpioAlternateFunction<Port::B, 9>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM4_CH4      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM4_CH4),
      TIM11_CH1     =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM11_CH1),
      I2C1_SDA      =   static_cast<uint16_t>(GpioAlternateFunctionId::I2C1_SDA),
      SPI2_NSS      =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI2_NSS),
      I2S2_WS       =   static_cast<uint16_t>(GpioAlternateFunctionId::I2S2_WS),
      SDIO_D5       =   static_cast<uint16_t>(GpioAlternateFunctionId::SDIO_D5),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM4_CH4, TIM11_CH1, I2C1_SDA, SPI2_NSS, I2S2_WS, SDIO_D5, EVENT_OUT};
  };

  template<>
  struct GpioAlternateFunction<Port::B, 10>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM2_CH3      =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM2_CH3),
      I2C2_SCL      =   static_cast<uint16_t>(GpioAlternateFunctionId::I2C2_SCL),
      SPI2_SCK      =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI2_SCK),
      I2S2_CK       =   static_cast<uint16_t>(GpioAlternateFunctionId::I2S2_CK),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM2_CH3, I2C2_SCL, SPI2_SCK, I2S2_CK, EVENT_OUT};
  };

  template<>
  struct GpioAlternateFunction<Port::B, 12>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM1_BKIN     =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM1_BKIN),
      I2C2_SMBA     =   static_cast<uint16_t>(GpioAlternateFunctionId::I2C2_SMBA),
      SPI2_NSS      =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI2_NSS),
      I2S2_WS       =   static_cast<uint16_t>(GpioAlternateFunctionId::I2S2_WS),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM1_BKIN, I2C2_SMBA, SPI2_NSS, I2S2_WS, EVENT_OUT};
  };

  template<>
  struct GpioAlternateFunction<Port::B, 13>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM1_CH1N     =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM1_CH1N),
      SPI2_SCK      =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI2_SCK),
      I2S2_CK       =   static_cast<uint16_t>(GpioAlternateFunctionId::I2S2_CK),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM1_CH1N, SPI2_SCK, I2S2_CK, EVENT_OUT};
  };
  
  template<>
  struct GpioAlternateFunction<Port::B, 14>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      TIM1_CH2N     =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM1_CH2N),
      SPI2_MISO     =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI2_MISO),
      I2S2ext_SD      =   static_cast<uint16_t>(GpioAlternateFunctionId::I2S2ext_SD),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, TIM1_CH2N, SPI2_MISO, I2S2ext_SD, EVENT_OUT};
  };
  
  template<>
  struct GpioAlternateFunction<Port::B, 15>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      RTC_REFN      =   static_cast<uint16_t>(GpioAlternateFunctionId::RTC_REFN),
      TIM1_CH3N     =   static_cast<uint16_t>(GpioAlternateFunctionId::TIM1_CH3N),
      SPI2_MOSI      =   static_cast<uint16_t>(GpioAlternateFunctionId::SPI2_MOSI),
      I2S2_SD       =   static_cast<uint16_t>(GpioAlternateFunctionId::I2S2_SD),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, RTC_REFN, TIM1_CH3N, SPI2_MOSI, I2S2_SD, EVENT_OUT};
  };

  template<>
  struct GpioAlternateFunction<Port::C, 13>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, EVENT_OUT};
  };

  template<>
  struct GpioAlternateFunction<Port::C, 14>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, EVENT_OUT};
  };

  template<>
  struct GpioAlternateFunction<Port::C, 15>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, EVENT_OUT};
  };

  template<>
  struct GpioAlternateFunction<Port::H, 0>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, EVENT_OUT};
  };

  template<>
  struct GpioAlternateFunction<Port::H, 1>
  {
    enum Type : uint16_t
    {
      Default       =   static_cast<uint16_t>(GpioAlternateFunctionId::Default),
      EVENT_OUT     =   static_cast<uint16_t>(GpioAlternateFunctionId::EVENTOUT)
    };

    static constexpr Type types[] = {Default, EVENT_OUT};
  };
}

#endif //STM32_GPIO_ALTERNATE_FUNCTIONS_HPP