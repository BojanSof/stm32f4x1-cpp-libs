cmake_minimum_required(VERSION 3.18)

set(TARGET "stm32f4x1-devices")

add_library(${TARGET})
target_sources(${TARGET}
  PUBLIC
    "include/STM32F4x1/Gpio.hpp"
    "include/STM32F4x1/GpioTypes.hpp"
    "include/STM32F4x1/GpioAlternateFunctions.hpp"
    "include/STM32F4x1/ExternalInterruptController.hpp"
    "include/STM32F4x1/Clock.hpp"
    "include/STM32F4x1/CycleCounter.hpp"
    "include/STM32F4x1/GeneralPurposeTimer.hpp"
    "include/STM32F4x1/ComInterface.hpp"
    "include/STM32F4x1/I2C.hpp"
    "include/STM32F4x1/SPI.hpp"
    "include/STM32F4x1/SerialUsb.hpp"
  PRIVATE
    "src/STM32F4x1/Clock.cpp"
    "src/STM32F4x1/GeneralPurposeTimer.cpp"
    "src/STM32F4x1/ExternalInterruptController.cpp"
    "src/STM32F4x1/I2C.cpp"
    "src/STM32F4x1/SPI.cpp"
    "src/STM32F4x1/SerialUsb.cpp"
)

target_include_directories(${TARGET}
  PRIVATE
    src
  PUBLIC
    include
)

target_compile_features(${TARGET}
  PRIVATE
    cxx_std_17)

target_link_libraries(${TARGET}
  PUBLIC
    ${CMSIS_LIBRARY}
    STM32::NoSys
    STM32::Nano
    TinyUsbCdc
)