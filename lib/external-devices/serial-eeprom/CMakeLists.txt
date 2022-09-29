cmake_minimum_required(VERSION 3.18)

set(TARGET "serial-eeprom")

add_library(${TARGET} INTERFACE)
target_sources(${TARGET}
  INTERFACE
    "include/SerialEeprom.hpp"
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
)