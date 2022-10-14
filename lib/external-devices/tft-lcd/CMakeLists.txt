cmake_minimum_required(VERSION 3.18)

set(TARGET "tft-lcd")

add_library(${TARGET} INTERFACE)
target_sources(${TARGET}
  INTERFACE
    "include/TftLcd.hpp"
    "include/TftLcdCommands.hpp"
    "include/TftLcdGpio.hpp"
    "include/TftLcdSpi.hpp"
    "include/Touch.hpp"
    "include/TouchCalibration.hpp"
)

target_include_directories(${TARGET}
  INTERFACE
    include
)

target_compile_features(${TARGET}
  INTERFACE
    cxx_std_17)

target_link_libraries(${TARGET}
  INTERFACE
    stm32f4x1-devices
    embedded-gfx
)