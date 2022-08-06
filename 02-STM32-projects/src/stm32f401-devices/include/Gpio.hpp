#ifndef STM32F401_GPIO_HPP
#define STM32F401_GPIO_HPP

#include <stm32f4xx.h>
#include <cstdint>

namespace Stm32
{

  /**
   * Possible GPIO ports for STM32F401CCU6,
   * UQFN48 package
   * 
   */
  enum class Port
  {
    A,
    B,
    C,
    H
  };

  enum class GpioMode : uint8_t
  {
    Input = 0b00,
    Output = 0b01,
    Alternate = 0b10,
    Analog = 0b11
  };

  enum class GpioSpeed : uint8_t
  {
    Low = 0b00,
    Medium = 0b01,
    High = 0b10,
    VeryHigh = 0b11
  };

  enum class GpioOutputType : uint8_t
  {
    PushPull = 0,
    OpenDrain = 1
  };

  enum class GpioPullType : uint8_t
  {
    None = 0b00,
    PullUp = 0b01,
    PullDown = 0b10,
    Reserved = 0b11
  };

  enum class GpioAlternateFunctionNumber : uint8_t
  {
    Af0 = 0x0,
    Af1 = 0x1,
    Af2 = 0x2,
    Af3 = 0x3,
    Af4 = 0x4,
    Af5 = 0x5,
    Af6 = 0x6,
    Af7 = 0x7,
    Af8 = 0x8,
    Af9 = 0x9,
    Af10 = 0xA,
    Af11 = 0xB,
    Af12 = 0xC,
    Af13 = 0xD,
    Af14 = 0xE,
    Af15 = 0xF
  };

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
      Default = GpioAlternateFunctionId::Default
    } type_;
  };

  template <Port port, uint8_t pin>
  class Gpio
  {
  public:
    using AlternateFunctions = GpioAlternateFunction<port, pin>;
    static constexpr Port pinPort = port;
    static constexpr uint8_t pinNumber = pin;

    Gpio()
    {
      static_assert(port == Port::A || port == Port::B ||
                    port == Port::C || port == Port::H,
                    "The specified port is not defined");

      // clock for the port
      if constexpr (port == Port::A)
      {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
      }
      else if constexpr (port == Port::B)
      {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
      }
      else if constexpr (port == Port::C)
      {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
      }
      else if constexpr (port == Port::H)
      {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
      }
    }

    void setMode(const GpioMode &mode)
    {
      gpioInstance_->MODER &= ~(0x3 << (2 * pin));
      gpioInstance_->MODER |= static_cast<uint8_t>(mode) << (2 * pin);
    }

    void setLevel(const bool lvl)
    {
      gpioInstance_->BSRR = 1 << (pin + (lvl ? 0 : 16));
    }

    bool getState()
    {
      return (gpioInstance_->IDR & (1 << pin));
    }

    void setSpeed(const GpioSpeed &speed)
    {
      gpioInstance_->OSPEEDR &= ~(0x3 << (2 * pin));
      gpioInstance_->OSPEEDR |= static_cast<uint8_t>(speed) << (2 * pin);
    }

    void setOutputType(const GpioOutputType &output)
    {
      gpioInstance_->OTYPER |= static_cast<uint8_t>(output) << pin;
    }

    void setPullType(const GpioPullType &pull)
    {
      gpioInstance_->PUPDR &= ~(0x3 << (2 * pin));
      gpioInstance_->PUPDR |= static_cast<uint8_t>(pull) << (2 * pin);
    }

    void setAlternateFunction(const GpioAlternateFunctionNumber &af)
    {
      setMode(GpioMode::Alternate);
      if constexpr (pin < 8)
      {
        gpioInstance_->AFR[0] |= static_cast<uint8_t>(af) << (4 * pin);
      }
      else
      {
        gpioInstance_->AFR[1] |= static_cast<uint8_t>(af) << (4 * pin);
      }
    }

    void setAlternateFunction(const typename AlternateFunctions::Type af)
    {
      setAlternateFunction(static_cast<GpioAlternateFunctionNumber>(static_cast<uint16_t>(af) >> 8));
    }

  private:
    constexpr static GPIO_TypeDef *getGpioInstance()
    {
      switch (port)
      {
      case Port::A:
        return GPIOA;
        break;
      case Port::B:
        return GPIOB;
        break;
      case Port::C:
        return GPIOC;
        break;
      case Port::H:
        return GPIOH;
        break;
      default:
        return nullptr;
        break;
      }
    }
    GPIO_TypeDef *const gpioInstance_ = getGpioInstance();
  };

  // Specialize the alternate functions template class for each pin
  template<>
  struct GpioAlternateFunction<Port::A, 0>
  {
    enum Type : uint16_t
    {
      Default       =   GpioAlternateFunctionId::Default,
      TIM2_CH1      =   GpioAlternateFunctionId::TIM2_CH1,
      TIM5_CH1      =   GpioAlternateFunctionId::TIM5_CH1,
      USART2_CTS    =   GpioAlternateFunctionId::USART2_CTS,
      EVENT_OUT     =   GpioAlternateFunctionId::EVENTOUT
    } type_;
  };
  ///@todo Specialize for other pins

  // Pins on STM32F401CCU6, UQFN48 package
  namespace Pins
  {
    using PA0 = Gpio<Port::A, 0>;
    using PA1 = Gpio<Port::A, 1>;
    using PA2 = Gpio<Port::A, 2>;
    using PA3 = Gpio<Port::A, 3>;
    using PA4 = Gpio<Port::A, 4>;
    using PA5 = Gpio<Port::A, 5>;
    using PA6 = Gpio<Port::A, 6>;
    using PA7 = Gpio<Port::A, 7>;
    using PA8 = Gpio<Port::A, 8>;
    using PA9 = Gpio<Port::A, 9>;
    using PA10 = Gpio<Port::A, 10>;
    using PA11 = Gpio<Port::A, 11>;
    using PA12 = Gpio<Port::A, 12>;
    using PA13 = Gpio<Port::A, 13>;
    using PA14 = Gpio<Port::A, 14>;
    using PA15 = Gpio<Port::A, 15>;

    using PB0 = Gpio<Port::B, 0>;
    using PB1 = Gpio<Port::B, 1>;
    using PB2 = Gpio<Port::B, 2>;
    using PB3 = Gpio<Port::B, 3>;
    using PB4 = Gpio<Port::B, 4>;
    using PB5 = Gpio<Port::B, 5>;
    using PB6 = Gpio<Port::B, 6>;
    using PB7 = Gpio<Port::B, 7>;
    using PB8 = Gpio<Port::B, 8>;
    using PB9 = Gpio<Port::B, 9>;
    using PB10 = Gpio<Port::B, 10>;
    using PB12 = Gpio<Port::B, 12>;
    using PB13 = Gpio<Port::B, 13>;
    using PB14 = Gpio<Port::B, 14>;
    using PB15 = Gpio<Port::B, 15>;

    using PC13 = Gpio<Port::C, 13>;
    using PC14 = Gpio<Port::C, 14>;
    using PC15 = Gpio<Port::C, 15>;

    using PH0 = Gpio<Port::H, 0>;
    using PH1 = Gpio<Port::H, 1>;
  }

}

#endif // STM32F401_GPIO_HPP