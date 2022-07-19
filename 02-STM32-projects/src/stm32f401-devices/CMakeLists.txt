cmake_minimum_required(VERSION 3.18)

set(TARGET "stm32f401-devices")

add_library(${TARGET})
target_sources(${TARGET}
    PRIVATE
      "include/Gpio.hpp"
      "include/Clock.hpp"

      "src/Clock.cpp"
)

target_include_directories(${TARGET}
    PRIVATE
      src
    PUBLIC
      include
)

target_compile_features(${TARGET}
  PUBLIC
    cxx_std_17)

target_link_libraries(${TARGET}
  PUBLIC
    CMSIS::STM32::F401CC
    STM32::NoSys)