#ifndef STM32_REAL_TIME_CLOCK_HPP
#define STM32_REAL_TIME_CLOCK_HPP

#include <stm32f4xx.h>
#include <chrono>
#include <ctime>
#include <cstdint>

namespace Stm32
{
  class RealTimeClock
  {
    public:
      using duration = std::chrono::seconds;
      using rep = duration::rep;
      using period = duration::period;
      using time_point = std::chrono::time_point<RealTimeClock>;
      static constexpr bool is_steady = true;

      static void init()
      {
        // clock config
        // enable PWR clock
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        // allow writing in backup domain registers
        PWR->CR |= PWR_CR_DBP;
        // reset backup domain
        RCC->BDCR |= RCC_BDCR_BDRST;
        RCC->BDCR &= ~RCC_BDCR_BDRST;
        // turn on LSE clock
        RCC->BDCR |= RCC_BDCR_LSEON;
        // wait for LSE clock to stabilize
        while(!(RCC->BDCR & RCC_BDCR_LSERDY));
        // set LSE as RTC clock source
        RCC->BDCR |= 0x01 << RCC_BDCR_RTCSEL_Pos;
        // enable RTC clock
        RCC->BDCR |= RCC_BDCR_RTCEN;

        // RTC config
        // enable writing to RTC registers
        enableRtcRegWrite();
        // LSE is 32.768 kHz, set async prescaler to 128, sync prescaler to 256
        // to achieve 1 Hz RTC clock
        RTC->PRER |= 127 << RTC_PRER_PREDIV_A_Pos;
        RTC->PRER |= 255 << RTC_PRER_PREDIV_S_Pos;
        // disable writing to RTC registers
        disableRtcRegWrite();
      }

      static void deinit()
      {
        // disable LSE clock
        RCC->BDCR &= ~RCC_BDCR_LSEON;
        // wait for LSE clock to disable
        while(RCC->BDCR & RCC_BDCR_LSERDY);
      }

      static time_point now() noexcept
      {
        // read date and time registers
        const uint32_t datereg = RTC->DR;
        const uint32_t timereg = RTC->TR;
        // store the time info
        std::tm timeinfo{};
        timeinfo.tm_wday = ((datereg & RTC_DR_WDU) >> RTC_DR_WDU_Pos) % 7;
        timeinfo.tm_mon  = bcdToDec((datereg & (RTC_DR_MT | RTC_DR_MU)) >> RTC_DR_MU_Pos) - 1;
        timeinfo.tm_mday = bcdToDec((datereg & (RTC_DR_DT | RTC_DR_DU)) >> RTC_DR_DU_Pos);
        timeinfo.tm_year = bcdToDec((datereg & (RTC_DR_YT | RTC_DR_YU)) >> RTC_DR_YU_Pos) + 100;  // for tm struct, year is the number of years since 1900, for stm32 is 00 to 99
        timeinfo.tm_hour = bcdToDec((timereg & (RTC_TR_HT | RTC_TR_HU)) >> RTC_TR_HU_Pos);
        timeinfo.tm_min  = bcdToDec((timereg & (RTC_TR_MNT | RTC_TR_MNU)) >> RTC_TR_MNU_Pos);
        timeinfo.tm_sec  = bcdToDec((timereg & (RTC_TR_ST | RTC_TR_SU)) >> RTC_TR_SU_Pos);
        return from_time_t(std::mktime(&timeinfo));
      }

      static void setTime(const time_point& t)
      {
        // convert timepoint to time info struct
        auto time = to_time_t(t);
        auto timeinfo = std::localtime(&time);
        timeinfo->tm_mon += 1;
        timeinfo->tm_year -= 100;
        if(timeinfo->tm_wday == 6) timeinfo->tm_wday = 7;
        // convert time info struct to register values
        const uint32_t timereg{
            decToBcd(timeinfo->tm_hour) << RTC_TR_HU_Pos
          | decToBcd(timeinfo->tm_min) << RTC_TR_MNU_Pos
          | decToBcd(timeinfo->tm_sec) << RTC_TR_SU_Pos
        };
        const uint32_t datereg{
            decToBcd(timeinfo->tm_mday) << RTC_DR_DU_Pos
          | decToBcd(timeinfo->tm_mon) << RTC_DR_MU_Pos
          | decToBcd(timeinfo->tm_year) << RTC_DR_YU_Pos
          | decToBcd(timeinfo->tm_wday) << RTC_DR_WDU_Pos
        };
        enableRtcRegWrite();
        // enter RTC init mode
        RTC->ISR |= RTC_ISR_INIT;
        while(!(RTC->ISR & RTC_ISR_INITF));
        // set RTC date
        RTC->DR = datereg;
        // set RTC time
        RTC->TR = timereg;
        // exit init mode
        RTC->ISR &= ~RTC_ISR_INIT;
        disableRtcRegWrite();
      }

      static std::time_t to_time_t(const time_point& t)
      {
        return std::chrono::duration_cast<std::chrono::duration<time_t>>(t.time_since_epoch()).count();
      }

      static time_point from_time_t(time_t t)
      {
        return time_point{
          std::chrono::duration_cast<duration>(std::chrono::duration<time_t>{t})
        };
      }
    private:
      static uint8_t decToBcd(const uint8_t dec)
      {
        return ((dec / 10) << 4) | (dec % 10);
      }

      static uint8_t bcdToDec(const uint8_t bcd)
      {
        return (bcd >> 4) * 10 + (bcd & 0xF);
      }

      static void enableRtcRegWrite()
      {
        RTC->WPR = 0xCA;
        RTC->WPR = 0x53;
      }

      static void disableRtcRegWrite()
      {
        RTC->WPR = 0xFF;
      }
  };
}

#endif //STM32_REAL_TIME_CLOCK_HPP