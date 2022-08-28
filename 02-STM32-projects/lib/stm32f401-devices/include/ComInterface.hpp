#ifndef COM_INTERFACE_HPP
#define COM_INTERFACE_HPP

#include <cstddef>
#include <functional>

namespace Stm32
{
  /**
   * Interface class for every communication protocol.
   * 
   */
  class ComInterface
  {
    public:
      using CallbackT = std::function<void()>;

      /**
       * @brief Set the callback function to handle
       * possible errors during async transfer
       * 
       * @param callback The callback function
       */
      virtual void setErrorCallback(const CallbackT &callback) = 0;

      /**
       * @brief Read requested number of bytes and store them
       * in a user-defined buffer via communication interface.
       * After read is done, call the user-passed callback function. 
       * @note This function is non-blocking.
       * 
       * @param buffer The buffer in which the read bytes are stored.
       * @param bytesToRead The requested number of bytes to read.
       * @param actualRead The number of bytes that were actually read.
       * @param callback The callback function.
       * @return true The read operation was started successfully.
       * @return false There is a transfer in progress.
       */
      virtual bool asyncRead(std::byte * const buffer, const size_t bytesToRead, size_t& actualRead, const CallbackT& callback) = 0;
      
      /**
       * @brief Write the bytes from the user-defined buffer
       * via the communication interface. After write is done,
       * call the user-passed callback function.
       * @note This function is non-blocking.
       * 
       * @param buffer The buffer which holds the bytes to be written.
       * @param bytesToWrite The number of bytes to write.
       * @param actualWrite The actual number of bytes that were written.
       * @param callback The callback function.
       * @return true The write operation was started successfully.
       * @return false There is a transfer in progress.
       */
      virtual bool asyncWrite(const std::byte * const buffer, const size_t bytesToWrite, size_t& actualWrite, const CallbackT& callback) = 0;

      /**
       * @brief Read requested number of bytes and store them
       * in a user-defined buffer via communication interface.
       * @note This function is blocking.
       * 
       * @param buffer The buffer in which the read bytes are stored.
       * @param bytesToRead The requested number of bytes to read.
       * @return size_t The number of bytes that were actually read.
       */
      size_t read(std::byte * const buffer, const size_t bytesToRead)
      {
        size_t readSize = 0;
        volatile bool readDone = false;
        if(!asyncRead(buffer, bytesToRead, readSize, [&readDone](){ readDone = true; }))
        {
          return 0;
        }
        while(!readDone);
        return readSize;
      }

      /**
       * @brief Write the bytes from the user-defined buffer
       * via the communication interface.
       * @note This function is blocking.
       * 
       * @param buffer The buffer which holds the bytes to be written.
       * @param bytesToWrite The number of bytes to write.
       * @return size_t The actual number of bytes that were written.
       */
      size_t write(const std::byte * const buffer, const size_t bytesToWrite)
      {
        size_t writeSize = 0;
        volatile bool writeDone = false;
        if(!asyncWrite(buffer, bytesToWrite, writeSize, [&writeDone](){ writeDone = true; }))
        {
          return 0;
        }
        while(!writeDone);
        return writeSize;
      }
  };

}
#endif  //COM_INTERFACE_HPP