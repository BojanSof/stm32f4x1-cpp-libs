#ifndef COM_INTERFACE_HPP
#define COM_INTERFACE_HPP

#include <cstddef>
#include <functional>

namespace Stm32
{
  struct ComConfig {};
  class ComInterface
  {
    public:
      using CallbackT = std::function<void()>;

      virtual void setErrorCallback(const CallbackT &callback) = 0;
      virtual bool asyncRead(std::byte * const buffer, const size_t bytesToRead, size_t& actualRead, const CallbackT& callback) = 0;
      virtual bool asyncWrite(const std::byte * const buffer, const size_t bytesToWrite, size_t& actualWrite, const CallbackT& callback) = 0;

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