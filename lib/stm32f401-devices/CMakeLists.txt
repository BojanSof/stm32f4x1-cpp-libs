cmake_minimum_required(VERSION 3.18)

set(TARGET "stm32f401-devices")

add_library(${TARGET})
target_sources(${TARGET}
  PUBLIC
    "include/Gpio.hpp"
    "include/GpioTypes.hpp"
    "include/GpioAlternateFunctions.hpp"
    "include/ExternalInterruptController.hpp"
    "include/Clock.hpp"
    "include/CycleCounter.hpp"
    "include/GeneralPurposeTimer.hpp"
    "include/ComInterface.hpp"
    "include/I2C.hpp"
    "include/SPI.hpp"
  PRIVATE
    "src/Clock.cpp"
    "src/GeneralPurposeTimer.cpp"
    "src/ExternalInterruptController.cpp"
    "src/I2C.cpp" 
    "src/SPI.cpp"   
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
    #cfsm-generator
    $<IF:$<STREQUAL:${DEVICE},STM32F401CC>,CMSIS::STM32::F401CC,CMSIS::STM32::F411CE>
    STM32::NoSys
    STM32::Nano)