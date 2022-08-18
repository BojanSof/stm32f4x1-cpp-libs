#ifndef COM_INTERFACE_HPP
#define COM_INTERFACE_HPP

#include <cstddef>
#include <functional>

namespace Stm32
{
  class ComInterface
  {
    public:
      using CallbackT = std::function<void()>;

      virtual void setErrorCallback(const CallbackT &callback) = 0;
      virtual bool asyncRead(std::byte * const buffer, const size_t bufferSize, size_t& readSize, const CallbackT& callback) = 0;
      virtual bool asyncWrite(const std::byte * const buffer, const size_t bufferSize, size_t& writeSize, const CallbackT& callback) = 0;

      size_t read(std::byte * const buffer, const size_t bufferSize)
      {
        size_t readSize = 0;
        volatile bool readDone = false;
        asyncRead(buffer, bufferSize, readSize, [&readDone](){ readDone = true; });
        while(!readDone);
        return readSize;
      }

      size_t write(const std::byte * const buffer, const size_t bufferSize)
      {
        size_t writeSize = 0;
        volatile bool writeDone = false;
        asyncWrite(buffer, bufferSize, writeSize, [&writeDone](){ writeDone = true; });
        while(!writeDone);
        return writeSize;
      }
  };

}
#endif  //COM_INTERFACE_HPP