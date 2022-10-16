#ifndef STM32_SERIAL_USB_HPP
#define STM32_SERIAL_USB_HPP

#include <algorithm>
#include <chrono>
#include <cstddef>

#include <tusb.h>

#include "STM32F4x1/Gpio.hpp"
#include "STM32F4x1/CycleCounter.hpp"

namespace Stm32
{
  /**
   * Serial interface implementation using
   * USB CDC class
   */
  class SerialUsb
  {
    public:
      /**
       * @brief Get the SerialUsb instance
       * 
       * @return SerialUsb& The SerialUsb instance
       */
      static SerialUsb& getInstance();

      /**
       * @brief Read requested number of bytes
       * @note Operation is blocking
       * 
       * @param buffer The buffer where the read data is stored
       * @param size The number of bytes to read
       * @param timeout The timeout for the operation
       * @return size_t The number of bytes that was read
       */
      template< class Rep, class Period >
      size_t read(std::byte* buffer, const size_t size, const std::chrono::duration<Rep, Period>& timeout)
      {
        std::byte* const iBegin = buffer;
        std::byte* const iEnd = buffer + size;
        std::byte* iCursor = buffer;

        const auto startTime = CycleCounter::now();
        while((iCursor < iEnd) && ((CycleCounter::now() - startTime) < timeout))
        {
          const size_t availableCount = getNumberAvailableBytes();
          // update USB stack if no data is available for reading
          if (!availableCount)
          {
            update();
          }
          else
          {
            iCursor += static_cast<size_t>(tud_cdc_read(iCursor, std::min(static_cast<size_t>(iEnd - iCursor), availableCount)));
          }
        }
        return static_cast<size_t>(iCursor - iBegin);
      }

      /**
       * @brief Write bytes
       * @note Operation is blocking
       * 
       * @param buffer The buffer where the data to be written is stored
       * @param size The number of bytes to write
       * @param timeout The timeout for the operation
       * @return size_t The number of bytes that was written
       */
      template< class Rep, class Period >
      size_t write(const std::byte* const buffer, const size_t size, const std::chrono::duration<Rep, Period>& timeout)
      {
        const std::byte* const iBegin = buffer;
        const std::byte* const iEnd = iBegin + size;
        const std::byte* iCursor = buffer;
        const auto startTime = CycleCounter::now();

        iCursor += static_cast<size_t>(tud_cdc_write(iCursor, static_cast<uint32_t>(iEnd-iCursor)));
        // Write FIFO is full, run usb background to flush
        while ((iCursor < iEnd) && ((CycleCounter::now() - startTime) < timeout))
        {
          update();
          iCursor += static_cast<size_t>(tud_cdc_write(iCursor, static_cast<uint32_t>(iEnd-iCursor)));
        }
        // Flush the write operation
        tud_cdc_write_flush();
        return static_cast<size_t>(iCursor - iBegin);
      }

      /**
       * @brief Get the number of bytes that is currently available
       * for reading
       * 
       * @return size_t The number of bytes that can be read
       */
      size_t getNumberAvailableBytes()
      {
        return tud_cdc_available();
      }

      /**
       * Run usb stack background tasks
       * @note Must be called in a loop
       * 
       */
      void update()
      {
        tud_task();
      }
    private:
      SerialUsb();
      // singleton class
      SerialUsb(const SerialUsb&) = delete;
      SerialUsb(SerialUsb&&) = delete;
      void operator=(const SerialUsb&) = delete;
      void operator=(SerialUsb&&) = delete;
  };
}

#endif //STM32_SERIAL_USB_HPP