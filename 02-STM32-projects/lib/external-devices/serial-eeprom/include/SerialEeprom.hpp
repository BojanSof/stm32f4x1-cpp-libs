#ifndef SERIAL_EEPROM_HPP
#define SERIAL_EEPROM_HPP

#include <cstdint>
#include <cstddef>
#include <chrono>

#include <I2C.hpp>
#include <CycleCounter.hpp>

namespace Devices
{
  /**
   * @brief Interface for 24Cxx serial EEPROMs
   * 
   * @tparam MemorySize Size of the EEPROM, in kbits
   * @tparam I2CinterfaceT The I2C interface type
   * @tparam I2Caddress The I2C address of the EEPROM
   * @tparam SdaPinT SDA pin type
   * @tparam SclPinT SCL pin type
   */
  template<
      uint8_t MemorySize
    , typename I2CinterfaceT
    , uint8_t I2Caddress
    , typename SdaPinT
    , typename SclPinT>
  class SerialEeprom
  {
    public:
      SerialEeprom()
      {
        static_cast(  MemorySize == 1
                  ||  MemorySize == 2
                  ||  MemorySize == 4
                  ||  MemorySize == 8
                  ||  MemorySize == 16
                  ||  MemorySize == 32
                  ||  MemorySize == 64
                  ||  MemorySize == 128
                  ||  MemorySize == 256
                  ||  MemorySize == 512
                  , "EEPROM size must be between 1 and 512 kbits, in power of two");
      }

      /**
       * @brief Reads the requested number of bytes 
       * from the given start address, from the EEPROM
       * 
       * @param buffer Pointer to the buffer where read data needs to be stored
       * @param startAddress The start address to read from the EEPROM
       * @param bytesToRead The number of bytes to read
       * @return size_t The number of bytes actually read
       */
      size_t readBytes(std::byte * const buffer, const uint16_t startAddress, const size_t bytesToRead)
      {
        // check if the start address is valid
        ///@note it is not checked if the internal address counter will overflow
        if(!isAddressValid(startAddress)) return 0;

        using namespace Stm32;
        auto &i2cInstance = I2CinterfaceT::getInstance();
        // configure the i2c interface
        using eepromI2Cconfig = I2Cconfig<
                100000U,    // frequency
                I2Caddress, // address
                SdaPinT,    // SDA pin
                SclPinT     // SCL pin
              >;
        i2cInstance.configure<eepromI2Cconfig>();
        // write the memory start address
        const std::byte memAddrBuffer[] = {
            static_cast<std::byte>(startAddress >> 8)
          , static_cast<std::byte>(startAddress & 0xFF)
        };
        i2cInstance.write(memAddrBuffer, sizeof(memAddrBuffer));
        if(i2cInstance.getErrorStatus() != I2Cerror::NoError)
        {
          return 0;
        }
        // read the requested number of bytes
        return i2cInstance.read(buffer, bytesToRead);
      }

      /**
       * @brief Write the given number of bytes in the
       * EEPROM to a given start address.
       * 
       * @param buffer The data that needs to be written in the EEPROM
       * @param startAddress The start address for the EEPROM write operation
       * @param bytesToWrite The number of bytes to write to the EEPROM
       * @return size_t The number of bytes actually written
       */
      size_t writeBytes(const std::byte * const buffer, const uint16_t startAddress, const size_t bytesToWrite)
      {
        // check if the start address is valid
        ///@note it is not checked if the internal address counter will overflow
        if(!isAddressValid(startAddress)) return 0;

        using namespace Stm32;
        auto &i2cInstance = I2CinterfaceT::getInstance();
        // configure the i2c interface
        using eepromI2Cconfig = I2Cconfig<
                100000U,    // frequency
                I2Caddress, // address
                SdaPinT,    // SDA pin
                SclPinT     // SCL pin
              >;
        i2cInstance.configure<eepromI2Cconfig>();
        // write the address
        const std::byte memAddrBuffer[] = {
            static_cast<std::byte>(startAddress >> 8)
          , static_cast<std::byte>(startAddress & 0xFF)
        };
        i2cInstance.write(memAddrBuffer, sizeof(memAddrBuffer));
        if(i2cInstance.getErrorStatus() != I2Cerror::NoError)
        {
          return 0;
        }
        // write the data byte by byte
        ///@todo utilize page write mode
        using namespace std::chrono_literals;
        static constexpr auto writeDelay= 5ms;  //< max time to write the data, refer to datasheet, write cycle time
        for(size_t iByte = 0; iByte < bytesToWrite; ++iByte)
        {
          i2cInstance.write(buffer[iByte], 1);
          CycleCounter::delay(writeDelay);
        }
      }
    private:
      /// check if a given address for the EEPROM is valid
      bool isAddressValid(const uint16_t address)
      {
        static constexpr uint32_t maxAddress = (MemorySize*1024)/8 - 1;
        return (address <= maxAddress);
      }
  };
}

#endif //SERIAL_EEPROM_HPP